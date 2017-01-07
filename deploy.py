import subprocess
import shlex


def execute_cmd(cmd):
    subprocess.call(shlex.split(cmd))


if __name__ == '__main__':
    execute_cmd('gradle clean')
    execute_cmd('gradle build -x test --offline')
    execute_cmd('scp trajectory/build/libs/swarmview-trajectory-1.1.45.jar controller@charmer.local:/home/controller/rats/rats_ws/src/rats/BeSwarm/libs')
    execute_cmd('scp trajectory/build/libs/swarmview-trajectory-1.1.45.jar controller@supercharmer.local:/home/controller/rats/rats_ws/src/rats/BeSwarm/libs')
