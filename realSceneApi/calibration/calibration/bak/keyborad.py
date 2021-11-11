#from pynput.keyboard import Key,Controller
from pynput import keyboard
allkey=[]
'''
keyboard=Controller()

#Press and release space
keyboard.press(Key.space)
keyboard.release(Key.space)

keyboard.press('a')
keyboard.release('a')

# Type two upper case As
keyboard.press('A')
keyboard.release('A')
with keyboard.pressed(Key.shift):
    keyboard.press('a')
    keyboard.release('a')

keyboard.type('Hello world')
'''

def on_press(key):
    try:
        if key.char=="r":
            print('alphanumeric key {0} pressed'.format(
                key.char))
    except AttributeError:
        print('special key {0} pressed'.format(
            key))

def on_release(key):
    allkey.append(str(key))
    if "'r'" in allkey:
        print('{0} released'.format(key))
        allkey.clear()
    if key==keyboard.Key.esc:
        #stop listener
        return False

# Collect events until released
with keyboard.Listener(
        on_press=on_press,
        on_release=on_release) as listener:
    listener.join()