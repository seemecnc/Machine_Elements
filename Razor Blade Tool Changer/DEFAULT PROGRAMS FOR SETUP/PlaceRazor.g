; PLACE TOOL - RAZOR BLADE
; by SeeMeCNC 2019
; sub program to return the blade to holder
; no copyright - just give us shout
; 
G28							;home printer
G1 X113.5 Y-53 Z45 F10000	;move above tool holder
G1 Y-53.5 Z37.5 F3000		;move pre-place position
G1 Y-54.8 Z34.0				;place position
G1 Y-52 Z37					;extra move away about blade angle
G1 Y-30	F10000				;move out to place completely
G1 X0 Y0 Z300				;move away from tool rack
M99
;
