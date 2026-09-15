import os
import time

def waitMoveDone(cc_client, param):
    cc_client.sendHRLog(2,'plugin_appdemo.py waitMoveDone call')
    time.sleep(0.02)
    isMoving = True
    while True:
        time.sleep(0.3)
        # 获取点位列表是否仍在运行中
        results = cc_client.HRApp('AppDemo','IsPointListMoving,0')
        cc_client.sendHRLog(2, 'waitMoveDone, results len: ' + str(len(results)))
        cc_client.sendHRLog(2, 'waitMoveDone, results: ' + ','.join(results))
        if(len(results) == 5):
            try:
                isMoving = (1 == int(results[3]))
            except ValueError:
                cc_client.sendHRLog(2, 'waitMoveDone, value error: ' + results[1])
                break
        else:
            cc_client.sendHRLog(2, 'waitMoveDone, IsPointListMoving return invalid results')
            break
        
        cc_client.sendHRLog(2, 'waitMoveDone, isMoving: ' + str(isMoving))
        if isMoving:
            continue
        else:
            cc_client.sendHRLog(2, 'waitMoveDone, Moving finished')
            break
