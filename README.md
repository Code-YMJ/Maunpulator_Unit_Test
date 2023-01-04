# Manupulator_Unit_Test
## 좌표계는 오른손 법칙을 따른다.

로봇의 좌표는 로봇 중심으로 오른손 법칙을 따른다.

카매라는 카메라 중심으로 오른손 법칙을 따른다.

Angle 단위

jr_main_form :
Only Use Degree

jeus_manupulator :
Only Use Degree

jeus_dynamixel : 
Degree -> Pulse / Pulse -> Degree

jeus_kinematictool : 
func : Radian / Degree->Radian/ Radian -> Degree



Move Fuction 기능 구현 범위

jr_main_form :
Angle 계산은 여기서 다 구현!
Move 명령  jeus_manupulator's Queue에 cmd, angle로 던지기

jeus_manupulator :
Path Planing -> jeus_dynamixel에 Move 명령 -> monitoring

jeus_dynamixel :
메모리에 Move 명령 넣기 & Position 읽기

jeus_kinematictool :
useful tools!
