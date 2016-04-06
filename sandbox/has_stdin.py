#!/usr/bin/env python
'Tests out try_get_line() from the util module'

# System includes
import sys
import time

# Local includes
import util

def main():
    'Main entry point.  Tests out try_get_line() for exiting an infinite loop.'
    is_still_going = True
    print 'Type "quit<ENTER>" to quit'
    while is_still_going:
        time.sleep(0.5)      # Simulate work to do
        line = util.try_get_line(sys.stdin)
        if line == 'quit':
            is_still_going = False
        elif line != '':
            print 'Unrecognized command: ' + line

if __name__ == '__main__':
    main()
