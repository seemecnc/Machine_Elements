; Center Eject pushes part using razor off bed
; by SeeMeCNC 2019
; sub program to pickup the blade
; no copyright - just give us shout out
; 
;G28						;home printer
G1 X0 Y135 Z60 F10000		;move above
G1 Z0.1 F6000				;move razor to build plate
M190 R40					;wait for bed to cool
M140 S0						;turn off bed heat
G4 S360						;wait 6 minutes for bed cooling
G1 Y-60 F1000 				;push part off bed
G1 Y0 Z200 F10000			;up outta the way
M99
;
