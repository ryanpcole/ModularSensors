# Development of CS10X analog temp sensor module
I'm basing this on the CS500 module that is reads temp and RH using the ADS1115 ADC.

Sensor documentation:
http://s.campbellsci.com/documents/us/manuals/108.pdf
https://s.campbellsci.com/documents/us/product-brochures/b_107_108.pdf
https://www.campbellsci.com/108
https://www.campbellsci.com/faqs?v=736
https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Data+Sheet%7FSeries_II_Thermistor_100K6A1I%7FA%7Fpdf%7FEnglish%7FENG_DS_Series_II_Thermistor_100K6A1I_A.pdf%7FGA100K6A1IA

## TODOs
-[] write CS10X header file
    -[] add paramaters for unique calibration?
-[] Figure out bridge resisitor wiring
-[] Write CS10X cpp file 
    -[] use make sure I use the right bridge resistor and formula
    -[] add unique calibration?
-[] make sure vin is stable - doesn't matter what voltage, but need a very stable voltage
    - gonna use 3v3 for this


