import tensorflow.lite as tflite
import time
#import tflite_runtime.interpreter as tflite
import numpy as np

# python3 -m tf_lite_test.test_converted
# The official tensorflow lite package is broken on RPi, 
# try https://github.com/PINTO0309/Tensorflow-bin


obs_used = np.array([[-0.5662416,  0.8242393,  0.6782238]], dtype=np.float32)

interpreter = tflite.Interpreter(model_path="tflite/converted_model.tflite")

interpreter.allocate_tensors()

input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

print(input_details[0]['index'])
interpreter.set_tensor(input_details[0]['index'], obs_used)

interpreter.invoke()
a = time.time()
tflite_results = interpreter.get_tensor(output_details[0]['index'])
print(time.time()-a)
a = time.time()
for i in range(40):
    tflite_results = interpreter.get_tensor(output_details[0]['index'])
print((time.time()-a)/40)

print("Result:")
print(tflite_results)

print("Input details: ")
print(input_details)
print("Output details: ")
print(output_details)