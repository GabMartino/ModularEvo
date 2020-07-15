

from ModularEvoManager import GazeboMessageSubscriber
import asyncio


Subscriber = GazeboMessageSubscriber("localhost", 11345)
loop = asyncio.get_event_loop()
loop.run_until_complete(Subscriber.connect())



## 45 = DIRECT ENCODING, 46 = ACK
## 125 = INDIRECT ENCODING, 126 = ACK
## 86 = DIRECT ENCODING MODULAR, 87 = ACK
## 101 = INDIRECT ENCODING MODULAR, 102 = ACK

experiment = 101
loop.run_until_complete(Subscriber.executeTest(experiment))