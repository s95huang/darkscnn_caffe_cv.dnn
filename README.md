# darkscnn_caffe_cv.dnn
using cv:dnn caffe to load and inference darkscnn models



### bug 
```
terminate called after throwing an instance of 'cv::Exception'
  what():  OpenCV(4.5.5) /home/s95huang/Downloads/opencv-4.5.5/modules/dnn/src/dnn.cpp:3231: error: (-215:Assertion failed) biasLayerData->outputBlobsWrappers.size() == 1 in function 'fuseLayers'
```
* looks like model error 

* Apollo's method

Apollo deployed this caffe model by using TensorRT API to replace some components with equivalent CUDA ops.
For example, slice layer from caffe is rewritten using CUDA layer.

* Aborting further development



## ref

https://docs.opencv.org/4.5.5/d5/de7/tutorial_dnn_googlenet.html
https://github.com/ApolloAuto/apollo
