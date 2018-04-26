import random
import attr
import heapq
import argparse
import math
import multiprocessing as mp
from functools import partial
from itertools import chain


def dist(a, b):
    """
    Compute Euclidian distance between points ``a`` and ``b``.

    :param a: 2-``tuple``
    :param b: 2-``tuple``
    :return: The distance between ``a`` and ``b``
    :rtype: ``float``
    """
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def circle_angle(point, center=(0, 0)):
    """
    Compute the angle of a point along a circle, given a center
    ``center``. Results are in ``(-pi, pi]``
    """
    return math.atan2(point[1] - center[1], point[0] - center[0])


def tikzpos(p):
    """
    Return the representation of a coordinate in TikZ coordinates.
    """
    return '({},{})'.format(*p)


@attr.s(repr=False)
class Robot:
    id = attr.ib()
    position = attr.ib(default=lambda t: ((0, 0), 0))
    initial_battery_level = attr.ib(default=100.0)
    battery_degrade_factor = attr.ib(default=0.1, type=float)
    travel_speed = attr.ib(default=1.0, type=float)
    distance_travelled = attr.ib(default=0.0, type=float)
    assigned_task = attr.ib(default=None)
    arrival_event = attr.ib(default=None, init=False)
    rosrobot = attr.ib(default=None, init=False)

    @property
    def battery_level(self):
        """
        The battery level of the robot.
        """
        return (self.initial_battery_level
                - self.battery_degrade_factor
                * self.distance_travelled)

    def travel(self, current_time, pos):
        """
        Update position function to reflect travel to a new position,
        and return the time at which we will reach that position.
        """
        cur_pos, travelled = self.position(current_time)
        self.distance_travelled += travelled

        if self.arrival_event:
            self.arrival_event.cancelled = True

        d = dist(cur_pos, pos)

        def position(t):
            if d == 0:
                return pos, 0
            completed = self.travel_speed * (t - current_time) / d
            if completed >= 1:
                return pos, d
            return (tuple(
                completed * (f - i) + i
                for f, i in zip(pos, cur_pos)), completed * d)

        self.position = position
        return d * self.travel_speed + current_time

    def tikz_repr(self, state):
        return r'''
            \node[draw,fill,circle,color=black,text=white,scale=0.5]
            (Robot{}) at {} {{{}}};'''.format(
                self.id,
                tikzpos(self.position(state.clock)[0]),
                self.id)

    def __repr__(self):
        return 'Robot{}'.format(self.id)


@attr.s(repr=False)
class Task:
    id = attr.ib()
    position = attr.ib(default=(0, 0))
    radius = attr.ib(default=1.0, type=float)
    assigned_robots = attr.ib(default=attr.Factory(list))
    present_robots = attr.ib(default=attr.Factory(list))

    def tikz_repr(self, state):
        return r'''
            \node[draw,fill,circle,color=red,text=white]
            (Task{}) at {} {{{}}};'''.format(
                self.id,
                tikzpos(self.position),
                self.id)

    def __repr__(self):
        return 'Task{}'.format(self.id)


class ROSRobot(mp.Process):
    """
    A connection to a single ROS turtlesim robot, with a ROS node
    running in its own process.
    """
    def __init__(self, robot_id, initpos):
        self.queue = mp.Queue(maxsize=1)
        self.initpos = initpos
        self.turtlename = 'turtle{}'.format(robot_id)
        super().__init__()

    def run(self):
        import rospy
        import turtlesim.srv
        self.rospy = rospy
        rospy.init_node(self.turtlename)

        rospy.wait_for_service('kill')
        kill = rospy.ServiceProxy(
            'kill',
            turtlesim.srv.Kill)
        try:
            kill(self.turtlename)
        except rospy.service.ServiceException:
            pass

        rospy.wait_for_service('spawn')
        spawn = rospy.ServiceProxy(
            'spawn',
            turtlesim.srv.Spawn)

        spawn(self.initpos[0], self.initpos[1], 0, self.turtlename)

        rospy.wait_for_service('{}/set_pen'.format(self.turtlename))
        pen = rospy.ServiceProxy(
            '{}/set_pen'.format(self.turtlename),
            turtlesim.srv.SetPen)
        pen(0, 0, 0, 0, 1)

        rospy.wait_for_service('{}/teleport_absolute'.format(self.turtlename))
        tp = rospy.ServiceProxy(
            '{}/teleport_absolute'.format(self.turtlename),
            turtlesim.srv.TeleportAbsolute)

        while True:
            tpargs = self.queue.get()
            tp(*tpargs)


class ROSNumberDrawer(mp.Process):
    """
    A ROS robot for drawing numbers at task locations.
    """

    digits = {
        0: {0, 1, 2, 3, 4, 5},
        1: {0, 5},
        2: {1, 0, 6, 3, 4},
        3: {4, 5, 6, 0, 1},
        4: {2, 6, 0, 5},
        5: {1, 2, 6, 5, 4},
        6: {1, 2, 6, 5, 4, 3},
        7: {1, 0, 5},
        8: {0, 1, 2, 3, 4, 5, 6},
        9: {1, 2, 6, 0, 5, 4},
        10: {1, 2, 6, 0, 3, 5},
        11: {2, 3, 4, 5, 6},
        12: {1, 2, 3, 4},
        13: {0, 5, 4, 3, 6},
        14: {1, 2, 3, 4, 6},
        15: {3, 6, 2, 1},
    }

    segpos = [
        (0.5, 1),
        (-0.5, 1),
        (-0.5, 0),
        (-0.5, -1),
        (0.5, -1),
        (0.5, 0),
        (-0.5, 0),
    ]

    def __init__(self, digit, center, segwidth=0.3):
        self.center = center
        self.digit = digit
        self.segwidth = segwidth
        self.turtlename = 'turtle{}'.format(1000 + digit)
        super().__init__()

    def run(self):
        import rospy
        import turtlesim.srv
        self.rospy = rospy
        rospy.init_node(self.turtlename)

        rospy.wait_for_service('kill')
        kill = rospy.ServiceProxy(
            'kill',
            turtlesim.srv.Kill)
        try:
            kill(self.turtlename)
        except rospy.service.ServiceException:
            pass

        rospy.wait_for_service('spawn')
        spawn = rospy.ServiceProxy(
            'spawn',
            turtlesim.srv.Spawn)

        spawn(self.center[0] + self.segwidth / 2,
              self.center[1],
              math.pi / 2,
              self.turtlename)

        rospy.wait_for_service('{}/set_pen'.format(self.turtlename))
        pen = rospy.ServiceProxy(
            '{}/set_pen'.format(self.turtlename),
            turtlesim.srv.SetPen)

        rospy.wait_for_service('{}/teleport_absolute'.format(self.turtlename))
        tp = rospy.ServiceProxy(
            '{}/teleport_absolute'.format(self.turtlename),
            turtlesim.srv.TeleportAbsolute)

        for seg in range(7):
            if seg in ROSNumberDrawer.digits[self.digit]:
                pen(255, 255, 255, 1, 0)
            else:
                pen(0, 0, 0, 0, 1)
            sp = ROSNumberDrawer.segpos[seg]
            tp(*(
                [self.segwidth * sp[k] + self.center[k]
                    for k in (0, 1)] + [0]))

        kill(self.turtlename)


@attr.s(cmp=False)
class Event:
    """
    Base class for all events.
    """
    time = attr.ib(type=float)
    cancelled = attr.ib(type=bool, default=False, init=False)

    def __lt__(self, other):
        def order(e):
            return (e.time, isinstance(e, MetaEvent), id(e.__class__))
        return order(self) < order(other)


@attr.s(cmp=False)
class RobotCreated(Event):
    robot = attr.ib()

    def __call__(self, state):
        state.robots.append(self.robot)
        if state.ros:
            self.robot.rosrobot = ROSRobot(
                self.robot.id, (*self.robot.position(state.clock)[0], 0))
            self.robot.rosrobot.daemon = True
            self.robot.rosrobot.start()


@attr.s(cmp=False)
class TaskCreated(Event):
    task = attr.ib()

    def __call__(self, state):
        state.tasks.append(self.task)
        partitions = state.robot_partitioning_func(state)
        for task, robots in zip(state.tasks, partitions):
            robots = list(robots)
            task.assigned_robots = robots
            task.present_robots = []
            positions = state.task_position_func(task, robots)
            for robot, position in zip(robots, positions):
                robot.assigned_task = task
                state.push_event(
                    BeginTravelToTask(state.clock, robot, position, task))

        if state.ros:
            drawer = ROSNumberDrawer(self.task.id, self.task.position)
            drawer.start()
            drawer.join()


def iter_chunks(it, n=3):
    """
    Iterate over chunks of size ``n`` from the iterable ``it``.
    """
    it = iter(it)
    chunk = []
    while True:
        if len(chunk) == n:
            yield chunk
            chunk = []
        try:
            itm = next(it)
        except StopIteration:
            if chunk:
                yield chunk
            return
        chunk.append(itm)


def partition_stupid(state):
    """
    Stupid partitioning method: partition the robots evenly among all
    tasks, simply based on their order listed in the system state.

    Intentionally bad: gives us a good baseline.
    """
    it = iter_chunks(state.robots, len(state.robots) // len(state.tasks))
    for i, c in enumerate(it):
        if i < len(state.tasks) - 1:
            yield c
        else:
            # excess get shoved into last group
            yield chain(c, *it)


def position_static(task, robots):
    r"""
    Position the robots statically in a circle around the task. The
    first robot will be placed at :math:`0` radians, the second robot at
    :math:`\frac{2\pi}{k}` radians, ...
    """
    sep = (2 * math.pi) / len(robots)
    for x in range(len(robots)):
        theta = x * sep
        yield (task.position[0] + task.radius * math.cos(theta),
               task.position[1] + task.radius * math.sin(theta))


@attr.s(cmp=False)
class BeginTravelToTask(Event):
    """
    The robot begins their travel to a task.
    """
    robot = attr.ib()
    destination = attr.ib(type=tuple)
    dest_task = attr.ib()

    def __call__(self, state):
        arrival = self.robot.travel(state.clock, self.destination)
        ev = ArrivalAtTask(arrival, self.robot, self.dest_task)
        self.robot.arrival_event = ev
        state.push_event(ev)


@attr.s(cmp=False)
class ArrivalAtTask(Event):
    """
    The robot has (potentially) arrived at a task. Cancelled if the
    robot made other tavel plans in the mean time.

    If all robots have arrived at the task, create a :class:`TaskBegin`
    event.
    """
    robot = attr.ib()
    task = attr.ib()

    def __call__(self, state):
        self.task.present_robots.append(self.robot)
        if all(x in self.task.present_robots
               for x in self.task.assigned_robots):
            state.push_event(TaskBegin(state.clock, self.task))


@attr.s(cmp=False)
class TaskBegin(Event):
    """
    The robot has begun work on a task.
    """
    task = attr.ib()

    def __call__(self, state):
        def task_pos(start_pos, start_time, task_center, speed, t):
            radius = dist(task_center, start_pos)
            start_angle = circle_angle(start_pos, task_center)
            travel = speed * (t - start_time)
            dtheta = travel / radius
            end_angle = start_angle + dtheta
            return ((task_center[0] + radius * math.cos(end_angle),
                     task_center[1] + radius * math.sin(end_angle)),
                    travel)

        for robot in self.task.assigned_robots:
            assert robot.assigned_task is self.task
            cur_pos, travelled = robot.position(state.clock)
            robot.distance_travelled += travelled
            robot.position = partial(
                task_pos,
                cur_pos,
                state.clock,
                self.task.position,
                robot.travel_speed)


class MetaEvent(Event):
    """
    Base class for all metaevents.
    """
    pass


@attr.s(cmp=False)
class TikzDraw(MetaEvent):
    """
    Metaevent to output a frame of drawing for TikZ. Only occurs
    if the program was called with the ``--tikz`` flag.
    """
    def __call__(self, state):
        state.tikz.write(r'''
            \begin{{tikzpicture}}[scale=0.6]
            \draw[step=1.0,gray,very thin] (0,0) grid (12,12);
            \draw (12,12) node[fill=gray,anchor=north east] {{$t = {:.1f}$}};
            '''.format(state.clock))
        for obj in state.tasks + state.robots:
            state.tikz.write(obj.tikz_repr(state))
        state.tikz.write(r'\end{tikzpicture}' + '\n')
        if any(not isinstance(e, MetaEvent) for e in state.eventq):
            state.push_event(TikzDraw(state.clock + state.tikz_interval))


@attr.s(cmp=False)
class ROSDraw(MetaEvent):
    """
    Metaevent to draw on the ROS simulation.
    """
    def __call__(self, state):
        for robot in state.robots:
            pos, trav = robot.position(state.clock)
            posn, travn = robot.position(state.clock + state.ros)
            theta = circle_angle(posn, center=pos)
            robot.rosrobot.queue.put((*pos, theta))
        state.push_event(ROSDraw(state.clock + state.ros))


@attr.s
class SimState:

    robot_partitioning_func = attr.ib(default=partition_stupid)
    task_position_func = attr.ib(default=position_static)

    tikz = attr.ib(default=None)
    tikz_interval = attr.ib(default=1)

    ros = attr.ib(default=False, type=bool)

    clock = attr.ib(default=0.0, type=float)

    robots = attr.ib(default=attr.Factory(list), init=False)
    tasks = attr.ib(default=attr.Factory(list), init=False)

    battery_rng = attr.ib(default=attr.Factory(random.Random), init=False)

    # Event List
    eventq = attr.ib(init=False, default=attr.Factory(list))

    def push_event(self, ev):
        """ adds an Event to the event list """
        print("Queue @{}:".format(self.clock), ev)
        return heapq.heappush(self.eventq, ev)

    def pop_event(self):
        """ pops an Event from the top of the event list """
        ev = heapq.heappop(self.eventq)
        self.clock = ev.time
        print("Run:", ev)
        return ev


def run_sim(battery_seed,
            num_robots=6,
            scatter=False,
            task_creations=(),
            *args, **kwargs):
    state = SimState(*args, **kwargs)
    state.battery_rng = random.Random(battery_seed)

    for k in range(num_robots):
        initpos = (0.5, k + 0.5)
        if scatter:
            initpos = tuple(
                state.battery_rng.uniform(0.5, 11.5)
                for _ in range(2))
        state.push_event(
            RobotCreated(
                time=0,
                robot=Robot(
                    id=k + 1,
                    position=(lambda p: lambda t: (p, 0))(initpos),
                    initial_battery_level=state.battery_rng.uniform(95, 100))))

    for i, t in enumerate(task_creations):
        state.push_event(TaskCreated(t[0], Task(i + 1, t[1:])))

    if state.tikz:
        state.tikz.write(r'''
            \documentclass[multi=tikzpicture,crop,tikz]{standalone}
            \usetikzlibrary{shapes,arrows}
            \begin{document}''')
        state.push_event(TikzDraw(0))

    if state.ros:
        state.push_event(ROSDraw(0))

    while state.eventq:
        ev = state.pop_event()
        if ev.cancelled:
            print("Skipping cancelled event: {!r}".format(ev))
            continue
        ev(state)

    if state.tikz:
        state.tikz.write(r'\end{document}' + '\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-n",
        "--num-robots",
        type=int,
        default=6,
        help="Number of robots to initialize at beginning of sim")
    parser.add_argument(
        "--task-creations",
        type=lambda s: eval("[{}]".format(s)),
        default=[(1, 6, 8),
                 (18, 3, 9),
                 (25, 9, 3),
                 (40, 3, 3),
                 (52, 9, 9),
                 (68, 6, 4)],
        help="A comma separated list of (time, x, y) tuples")
    parser.add_argument(
        "--battery-seed",
        type=int,
        default=None,
        help="Seed for battery RNG")
    parser.add_argument(
        "--tikz",
        type=argparse.FileType('w'),
        default=None,
        help="Output TikZ file, and enables TikZ metaevents")
    parser.add_argument(
        "--tikz-interval",
        type=float,
        default=1.0,
        help="Interval between TikZ snapshots")
    parser.add_argument(
        "--scatter",
        action="store_true",
        default=False,
        help="Scatter robots")
    parser.add_argument(
        "--ros",
        type=float,
        default=0,
        help="ROS metaevent interval, 0 to disable (default)")

    args = parser.parse_args()
    run_sim(**vars(args))
