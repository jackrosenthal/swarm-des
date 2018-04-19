import random
import attr
import heapq
import argparse
import math
from functools import partial


def dist(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def circle_angle(point, center=(0, 0)):
    """
    Compute the angle of a point along a circle, given a center
    ``center``. Results are in (-pi, pi]
    """
    return math.atan2(point[1] - center[1], point[0] - center[0])


def tikzpos(p):
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

    @property
    def battery_level(self):
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


@attr.s(cmp=False)
class Event:
    time = attr.ib(type=float)
    def __lt__(self, other):
        def order(e):
            return (e.time, isinstance(e, MetaEvent), id(e.__class__))
        return order(self) < order(other)


@attr.s(cmp=False)
class RobotCreated(Event):
    robot = attr.ib()

    def __call__(self, state):
        state.robots.append(self.robot)


@attr.s(cmp=False)
class TaskCreated(Event):
    task = attr.ib()

    def __call__(self, state):
        state.tasks.append(self.task)
        partitions = state.robot_partitioning_func(state)
        for task, robots in zip(state.tasks, partitions):
            task.assigned_robots = list(robots)
            task.present_robots = []
            positions = state.task_position_func(task, robots)
            for robot, position in zip(robots, positions):
                robot.assigned_task = task
                state.push_event(
                    BeginTravelToTask(state.clock, robot, position, task))


def iter_chunks(it, n=3):
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
    yield from iter_chunks(state.robots, len(state.robots) // len(state.tasks))


def position_static(task, robots):
    sep = (2 * math.pi) / len(robots)
    for x in range(len(robots)):
        theta = x * sep
        yield (task.position[0] + task.radius * math.cos(theta),
               task.position[1] + task.radius * math.sin(theta))


@attr.s(cmp=False)
class BeginTravelToTask(Event):
    robot = attr.ib()
    destination = attr.ib(type=tuple)
    dest_task = attr.ib()

    def __call__(self, state):
        arrival = self.robot.travel(state.clock, self.destination)
        state.push_event(
            ArrivalAtTask(
                arrival,
                self.robot,
                self.dest_task))


@attr.s(cmp=False)
class ArrivalAtTask(Event):
    robot = attr.ib()
    task = attr.ib()

    def __call__(self, state):
        if self.robot.assigned_task != self.task:
            print("Dropping invalid event, the robot's task has changed")
            return
        self.task.present_robots.append(self.robot)
        if all(x in self.task.present_robots for x in self.task.assigned_robots):
            state.push_event(TaskBegin(state.clock, self.task))


@attr.s(cmp=False)
class TaskBegin(Event):
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
    pass


@attr.s(cmp=False)
class TikzDraw(MetaEvent):
    def __call__(self, state):
        state.tikz.write(r'''
            \begin{{tikzpicture}}[scale=0.6]
            \draw[step=1.0,gray,very thin] (0,0) grid (12,12);
            \draw (12,12) node[fill=gray,anchor=north east] {{$t = {:.1f}$}};
            '''
            .format(state.clock))
        for obj in state.tasks + state.robots:
            state.tikz.write(obj.tikz_repr(state))
        state.tikz.write(r'\end{tikzpicture}' + '\n')
        if any(not isinstance(e, MetaEvent) for e in state.eventq):
            state.push_event(TikzDraw(state.clock + state.tikz_interval))


@attr.s
class SimState:

    robot_partitioning_func = attr.ib(default=partition_stupid)
    task_position_func = attr.ib(default=position_static)

    tikz = attr.ib(default=None)
    tikz_interval = attr.ib(default=1)

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


def run_sim(battery_seed, num_robots=6, *args, **kwargs):
    state = SimState(*args, **kwargs)
    state.battery_rng = random.Random(battery_seed)

    for k in range(num_robots):
        state.push_event(
            RobotCreated(
                time=0,
                robot=Robot(
                    id=k + 1,
                    position=(lambda k: lambda t: ((0.5, k + 0.5), 0))(k),
                    initial_battery_level=state.battery_rng.uniform(95, 100))))

    tasks = [
        (1, Task(1, (6, 8))),
        (18, Task(2, (3, 9))),
        (25, Task(3, (9, 3))),
        (40, Task(4, (3, 3))),
        (52, Task(5, (9, 9))),
        (68, Task(6, (6, 4))),
    ]
    for t in tasks:
        state.push_event(TaskCreated(*t))

    if state.tikz:
        state.tikz.write(r'''
            \documentclass[multi=tikzpicture,crop,tikz]{standalone}
            \usetikzlibrary{shapes,arrows}
            \begin{document}''')
        state.push_event(TikzDraw(0))

    while state.eventq:
        state.pop_event()(state)

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

    args = parser.parse_args()
    run_sim(**vars(args))
