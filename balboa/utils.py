from inputs import devices
import inputs
import os
def tensorboard_latest_directory_number( path , str='PPO2_'):
    number = 0
    print(os.listdir(path))
    # List all folders
    for foldername in os.listdir(path):
        # If the folder name starts with the string (i.e. PPO_)
        if foldername.find(str) == 0:
            filename_number = int(foldername.replace(str, ''))
            print(filename_number)
            if number < filename_number:
                number = filename_number
    return number



if __name__ == '__main__':
    count = tensorboard_latest_directory_number('balboa/results')
    print(count)
    for device in devices:
        print(device)

    while 1:
        events = inputs.get_gamepad()
        for event in events:
            print(event.ev_type, event.code, event.state)