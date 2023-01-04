from jeus_vision import *
weight = 'btn_221203/best.pt'

test = jeus_vision()
test.init_camera()
test.init_yolo(weight)

# test.activate('person')
test.activate_test('btn_1')
test.stop()
