import torch
import numpy as np
import math
import datetime
import time
n = datetime.datetime.now()
time.sleep(0.5)
c = datetime.datetime.now()
print(n)
print(c)
print(c-n)
# dtype = torch.float
# # device = torch.device('cpu')
# device = torch.device('cuda:0') # GPU에서 실행하기 위해 이 주석을 제거하기
# l= np.linspace(-math.pi, math.pi, 2000)
# x = torch.tensor(np.linspace(-math.pi, math.pi, 2000), device=device, dtype=dtype)
# y = torch.tensor( np.sin(l), device=device, dtype=dtype)
# # 무작위 입력과 출력 데이터 생성하기
# a = torch.randn((), device=device, dtype=dtype)
# b = torch.randn((), device=device, dtype=dtype)
# c = torch.randn((), device=device, dtype=dtype)
# d = torch.randn((), device=device, dtype=dtype)

# learning_rate = 1e-6
# for t in range(10000):
#     # 순전파 : 예측된 y 계산하기
#     y_pred = a + b * x + c * x ** 2 + d * x ** 3
    
#     # 손실을 계산하고 추력하기
#     loss = (y_pred - y).pow(2).sum().item()
#     if t % 100 == 99:
#         print(t, loss)
        
#     # 손실에 대한 a, b, c, d의 변화도를 계산하기 위한 역전파
#     grad_y_pred = 2.0 * (y_pred - y)
#     grad_a = grad_y_pred.sum()
#     grad_b = (grad_y_pred * x).sum()
#     grad_c = (grad_y_pred * x ** 2).sum()
#     grad_d = (grad_y_pred * x ** 3).sum()
    
#     # 경사 하강법을 사용해서 가중치 갱신하기
#     a -= learning_rate * grad_a
#     b -= learning_rate * grad_b
#     c -= learning_rate * grad_c
#     d -= learning_rate * grad_d
    
# print(f'Result: y = {a.item()} + {b.item()} x + {c.item()} x^2 + {d.item()} x^3')