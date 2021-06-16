# Cereal

Serial messaging with Cap'n Proto and 0MQ in either Python or C++

Example
---
```python
import cereal.messaging as messaging

# in subscriber
sm = messaging.SubMaster(['sensorEvents'])
while 1:
  sm.update()
  print(sm['sensorEvents'])

```

```python
# in publisher
pm = messaging.PubMaster(['sensorEvents'])
dat = messaging.new_message('sensorEvents', size=1)
dat.sensorEvents[0] = {"gyro": {"v": [0.1, -0.1, 0.1]}}
pm.send('sensorEvents', dat)
```
