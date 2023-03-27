# darkscnn_caffe_cv.dnn
using cv:dnn caffe to load and inference darkscnn models with ros1 integration.
It doesn't need caffe modules if you compile CV with CUDA support.


### bug 
```
terminate called after throwing an instance of 'cv::Exception'
  what():  OpenCV(4.5.5) /home/s95huang/Downloads/opencv-4.5.5/modules/dnn/src/dnn.cpp:3231: error: (-215:Assertion failed) biasLayerData->outputBlobsWrappers.size() == 1 in function 'fuseLayers'
```
* looks like model error 

  * The output should be 1 X 13 X img_size. 
  * The cv.dnn deployment method causes dimmension mismatch.
  * Aborting further development


### Apollo's method

* Apollo deployed this caffe model by using TensorRT API to replace some components with equivalent CUDA ops.
  * For example, slice layer from caffe is rewritten using CUDA layer.

* Models are obtained from Apollo's Github repo.
  * Please refer to their website for licenses etc.

## ref

https://docs.opencv.org/4.5.5/d5/de7/tutorial_dnn_googlenet.html

https://github.com/ApolloAuto/apollo
