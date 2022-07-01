
Flash Program
FP 00000000 000 000000000 0000 //cmd addr size data crc
size: 001 - 100
Response:
- OK
- !SIZE ERROR
- !CRC ERROR
---
Flash Sector Erase
FE Start Num
FE 0 0
---
Flash Read Memory
FR 00000000 000 //cmd addr size
Response:
00000000 000 000000000 0000 // addr size data crc
size: 001 - 100