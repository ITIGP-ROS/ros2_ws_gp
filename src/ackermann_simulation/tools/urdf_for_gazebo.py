#!/usr/bin/env python3
"""Expand robot.xacro and REMOVE THE XML COMMENTS, for Gazebo only.

    urdf_for_gazebo.py <file.xacro> [name:=value ...]   -> URDF on stdout

WHY THIS EXISTS - a real failure, not tidiness
----------------------------------------------
gazebo_ros2_control 0.4.10 starts the controller_manager inside gzserver and hands it the
URDF as a COMMAND-LINE PARAMETER OVERRIDE: `--param robot_description:=<the entire urdf>`.
rcl parses the text after ':=' as YAML. A URDF is not YAML, and it only survives that
round trip by accident - as a multi-line plain scalar. Any line inside it that YAML reads
as structure ends the accident. The two that matter both occur naturally in prose:

    ... something: something          a colon followed by a space  -> a mapping entry
    ... something:                    a trailing colon             -> a mapping key

Neither can appear in URDF markup, where colons only show up inside attribute values such
as package:// or xmlns:xacro. They appear constantly in COMMENTS.

The failure mode is why this is worth a whole file. rcl rejects the argument, gzserver
logs one truncated line -

    [ERROR] [gazebo_ros2_control]: parser error Couldn't parse parameter override rule

- and then CARRIES ON. Gazebo runs, the model is there, the world looks right, and the
controller_manager never comes up, so every spawner sits at "waiting for service
/controller_manager/list_controllers to become available" until it times out two minutes
later. Nothing says the cause was a comment.

So the constraint is real and invisible: without this filter, ackermann_description's
files would carry an unwritten rule that a comment may not contain ': ' or end with ':',
enforced by nothing, discovered only by breaking the simulation. Comments are stripped
here instead, at the one place that needs them gone.

Comments are removed through the DOM rather than by regex, so a '-->' occurring inside an
attribute value cannot be mistaken for a comment terminator.

Only the SIMULATION uses this. The vehicle loads the URDF through robot_state_publisher
in the ordinary way and keeps every comment.
"""
import sys

import xacro


def strip_comments(node):
    for child in list(node.childNodes):
        if child.nodeType == child.COMMENT_NODE:
            node.removeChild(child)
        else:
            strip_comments(child)


def main():
    if len(sys.argv) < 2:
        sys.stderr.write(__doc__)
        return 2

    path = sys.argv[1]
    mappings = dict(a.split(':=', 1) for a in sys.argv[2:] if ':=' in a)

    doc = xacro.process_file(path, mappings=mappings)
    strip_comments(doc)
    urdf = doc.toprettyxml(indent='  ')
    # toprettyxml leaves the removed comments behind as blank lines.
    urdf = '\n'.join(line for line in urdf.splitlines() if line.strip())

    # Fail LOUDLY rather than let gzserver fail quietly. If markup ever does introduce one
    # of these sequences, the message names the line instead of leaving a two-minute
    # spawner timeout as the only symptom.
    for i, line in enumerate(urdf.splitlines(), 1):
        s = line.strip()
        if ': ' in s or s.endswith(':'):
            sys.stderr.write(
                'urdf_for_gazebo: line %d would break rcl parameter parsing and stop the\n'
                'controller_manager from starting (see this file\'s docstring):\n  %s\n'
                % (i, s))
            return 1

    sys.stdout.write(urdf)
    return 0


if __name__ == '__main__':
    sys.exit(main())
