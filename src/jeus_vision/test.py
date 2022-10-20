from jeus_vision import *
weight = 'weights/yolov5s.pt'
data = 'data/coco128.yaml'
test = jeus_vision()
test.init_camera()
test.init_yolo(weight)

test.activate('person')
# test.activate_test('person')
# test.stop()
