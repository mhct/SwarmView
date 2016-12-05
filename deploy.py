import subprocess
import shlex


def execute_cmd(cmd):
    subprocess.call(shlex.split(cmd))


if __name__ == '__main__':
    execute_cmd('gradle clean')
    execute_cmd('gradle build -x test')
    execute_cmd('scp trajectory/build/libs/swarmview-trajectory-1.1.45.jar virt-controller@virtual-charmer1:/home/virt-controller')