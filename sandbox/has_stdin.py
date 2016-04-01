#!/usr/bin/env python

import sys
import time
import select

def tryGetLine(inStream):
    'Returns a line if there is one, else an empty string'
    line = ''
    fd = inStream.fileno()
    readyToRead, _, _ = select.select([fd], [], [], 0.01)
    if fd in readyToRead:
        line = inStream.readline()
    return line[:-1]  # Remove the newline

def main():
    isStillGoing = True
    print('Type "quit<ENTER>" to quit')
    while isStillGoing:
        time.sleep(0.5)      # Simulate work to do
        line = tryGetLine(sys.stdin)
        if line == 'quit':
            isStillGoing = False
        elif line != '':
            print 'Unrecognized command: ' + line

if __name__ == '__main__':
    main()
