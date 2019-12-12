from inputs import devices
import inputs
import os
def tensorboard_latest_directory_number( path ):
    number = 0
    for filename in os.listdir(path):
        if filename.find('PPO2_') == 0:
            filename_number = int(filename.replace('PPO2_', ''))
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