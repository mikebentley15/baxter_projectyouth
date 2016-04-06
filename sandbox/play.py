
# coding: utf-8

# In[1]:

import sys
import select
import re


# The below imports will only work if you have ros and baxter tools installed and working, which isn't the case on my laptop.

# In[ ]:

import rospy
import baxter_interface


# I made this function when I was trying to make you type 'quit' to exit the command-line tool.  But I just caved and said 'Press &lt;CTRL&gt;-C to exit'

# In[2]:

def tryGetLine(inStream):
    'Returns a line if there is one, else an empty string'
    line = ''
    fd = inStream.fileno()
    timeout = 0.01 # seconds
    readyToRead, _, _ = select.select([fd], [], [], timeout)
    if fd in readyToRead:
        line = inStream.readline()
    return line[:-1]  # Remove the newline


# In[3]:

def connectToBaxter():
    pass


# In[4]:

def waitForButtonPress():
    pass


# This function will save basically any dictionary to a file in python notation.

# In[5]:

def saveJointAngles(filename, angles):
    '''
    Saves the angles dictionary to filename in python notation
    
    Will save it into an array of dictionaries called data.
    For example
    
    saveJointAngles('out.py', {'a': 1, 'b': 2})
    
    will write to 'out.py':
    data = []
    data.append({
        'a': 1,
        'b': 2,
        })
    '''
    string = str(angles)
    string = string.replace('{', '')
    string = string.replace('}', '')
    split = string.split(', ')
    split.append('})')
    isDataDefined = False
    try:
        with open(filename, 'r') as infile:
            for line in infile:
                if re.match('data =', line):
                    isDataDefined = True
                    break
    except IOError:
        pass # Ignore the problem that the file doesn't yet exist
    with open(filename, 'a') as outfile:
        if not isDataDefined:
            outfile.write('data = []\n')
        outfile.write('\n')
        outfile.write('data.append({\n    ')
        outfile.write(',\n    '.join(split))
        outfile.write('\n')


# In[6]:

def main():
    ''
    # Connect to Baxter
    leftArm, rightArm = connectToBaxter()
    
    # Record arm positions
    print 'Press <CTRL>-C to quit'
    while True:
        # Read button presses and see if we should record
        whichArm, action = waitForButtonPress()
        currentArm = leftArm if whichArm == 'left' else rightArm
        jointAngles = currentArm.joint_angles()
        saveJointAngles(whichArm + 'SavedJoints.py', jointAngles)


# In[ ]:

if __name__ == '__main__':
    sys.exit(main())


# First, let's make sure that out.py doesn't exist

# In[22]:

import os
try:
    os.remove('out.py')
except OSError:
    pass # Ignore if the file doesn't exist


# Now, check to see that I can call this multiple times and it creates only one data array with multiple dictionaries in it.

# In[28]:

saveJointAngles('out.py', {'a': 1.02, 'b': 3.333, 'c': True})
saveJointAngles('out.py', {'abc': 'jumbo shrimp', 'tree': 'tall', 'seed': True})


# In[29]:

with open('out.py', 'r') as infile:
    print infile.read()


# Let's see if we can import it and use it

# In[31]:

import out
reload(out)


# In[32]:

out.data


# In[ ]:



