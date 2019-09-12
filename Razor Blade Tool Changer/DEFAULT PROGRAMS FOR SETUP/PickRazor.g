; PICK TOOL - RAZOR BLADE
; by SeeMeCNC 2019
; sub program to pickup the blade
; no copyright - just give us shout
; 
G28							;home printer
G1 X113.5 Y-10 Z38 F10000	;move pre-pick position
G1 Y-52	F4000				;picking pre-pick
G1 Y-54 Z35					;black in pocket position
G1 Y-51.6 Z50 				;picked
M99
;
