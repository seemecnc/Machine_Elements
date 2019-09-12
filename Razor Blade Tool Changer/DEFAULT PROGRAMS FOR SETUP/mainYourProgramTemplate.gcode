; Main Program
;
; Calls sub programs ending in .g
;   >first print a part
;   >second eject the part
;
M98 P/gcodes/subNoTread.g
M98 P/gcodes/subEject[programname].g
;
M104 S0		;heat off
M140 S0		;bed off
M84			;motor off
;
