import time
from camera_wrapper_node import IntelRosWrapper
import select
import sys

# function used to stop loop functions
def stop_loop(stop_entry):
    '''
    Used to quit an infinite loop with a char/string entry
    '''
    rlist = select.select([sys.stdin], [], [], 0.001)[0]
    if rlist and sys.stdin.readline().find(stop_entry) != -1:
        return True
    return False

if __name__ == "__main__":

    camera = IntelRosWrapper()
    
    print("\nRunning\nTo quit the program press q and then Enter.")

    while not stop_loop('q'):

        camera.get_aligned_frames()
        camera.get_intrinsics()

