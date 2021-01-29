This is a pinmap of the veraboard prototype of the GPS tracker, Wireless Stick Lite.

GPIO33 is conn'd to the opamps output, for vibration sensor. Default high, will trigger low.
GPIO4 is conn't to the GPS module MOSFET (STP160N4LF6). High == GPS On, Low == GPS Off.
UART1 is conn't to the GPS module, init'd by Serial1.begin(9600);

