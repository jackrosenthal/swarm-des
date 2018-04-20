Swarm Robotics Discrete Event Simulation
========================================

:Authors: Jack Rosenthal and Qin Yang

This project includes a discrete event simulation of robots completing a task
by circling around the task.

System Requirements
-------------------

* A Linux workstation with Python 3.6 or greater
* The Python attrs_ library (``pip3 install attrs``)

.. _attrs: http://www.attrs.org/en/stable/

Running the Simulation
----------------------

The simplest usage is to type ``python3 swarmdes.py``. This will run the
simulation with the default parameters and no TikZ drawing, but the output is
not very readable (simply an output of each of the events, when they are
enqueued, and when they occur)

What is more useful is to run the simulation with the TikZ metaevents enabled.
To do so, use the following two command line parameters::

    --tikz=FILENAME.tex
    --tikz-interval=0.1     (or whatever time interval is desired)

This will draw a TikZ frame to ``FILENAME.tex`` at every interval. You can then
render a pdf using ``xelatex FILENAME.tex``. Open the PDF in a fairly fast PDF
viewer, put it into presentation mode, and hold down the right arrow key to
watch [1]_.

.. [1] It's a bit of an ugly hack, I know. It's a LaTeX "flipbook!"

Adjusting Simulation Parameters
-------------------------------

To adjust the number of robots, use the ``-n`` or ``--num-robots`` argument.

To adjust when and where the tasks appear, use the ``--task-creations``
argument, passing a comma separated list of 3-tuples containing
``(time, x, y)`` of each task. For example::

    --task-creations="(0.5,2,2),(2,6,6)"

In particular, selection of close-together tasks allows for observation of the
cancellation of events.

Other Notes
-----------

I tried to make these instructions as detailed and complete as possible, but if
something seems missing or you aren't able to get things working, you can feel
free to Email me with questions: jrosenth@mines.edu

