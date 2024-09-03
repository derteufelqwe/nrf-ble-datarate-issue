# nRF BLE data rate issue
During the creation of benchmarks for our BLE applications, I observed that the maximum transferrate is significantly
lower (almost a factor of 8) when transmitting from ``Central -> Peripheral`` using GATT write.
The other direction from ``Peripheral -> Central`` using GATT notify performs as expected. <br>
Now the really weired part is, that doing both in parallel (to create bidirectional traffic) increases the throughput
from ``Central -> Peripheral`` significantly.

I compared my results with the values published on the [Nordic documentation](https://infocenter.nordicsemi.com/index.jsp?topic=%2Fsds_s140%2FSDS%2Fs1xx%2Fble_data_throughput%2Fble_data_throughput.html).
There it looks like both directions (``Peripheral -> Central`` and ``Central -> Peripheral``) should have equal throughput.


# Setup
|                     |                       |
|---------------------|-----------------------|
| nRF Connect version | 2.5.1                 |
| Hardware            | nRF52840 DevKit 3.0.0 |


The hardware setup is shown in the image below.
![Image](./docs/problem.drawio.svg)

# Measurements
The table below shows a few parameters, which are consistent across all measurements

| Setting          | Value      |
|------------------|------------|
| Measurement span | 30 seconds |
| PHY              | 1M         |
| L2CAP MTU        | 517        |
| MTU              | 251        |


## Scenario 1
| Config              | Value                     |
|---------------------|---------------------------|
| Connection Interval | 100 ms                    |
| Payload size        | 200 Bytes                 |
| Send direction      | Peripheral &rarr; Central |

> => 720 kbps

## Scenario 2
| Config              | Value                     |
|---------------------|---------------------------|
| Connection Interval | 100 ms                    |
| Payload size        | 200 Bytes                 |
| Send direction      | Central &rarr; Peripheral |

> => 92 kbps 

## Scenario 3
| Config              | Value                        |
|---------------------|------------------------------|
| Connection Interval | 100 ms                       |
| Payload size        | 200 Bytes                    |
| Send direction      | Both (bidirectional traffic) |

> => 412 kbps (Peripheral &rarr; Central) <br>
> => 404 kbps (Central &rarr; Peripheral)


## Scenario n
The same behaviour can be observed with different connection intervals and payload sizes.

