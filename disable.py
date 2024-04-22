import fileinput
import sys

def toggle_rc_local_bool():
    for line in fileinput.input('/etc/rc.local', inplace=True):
        if '_init' in line and "$" not in line:
            # Toggle the boolean value
            line = line.replace('false', 'true') if 'false' in line else line.replace('true', 'false')
        sys.stdout.write(line)

toggle_rc_local_bool()