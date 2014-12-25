
led.out:     file format elf32-avr


Disassembly of section .text:

00000000 <__vectors>:
   0:	09 c0       	rjmp	.+18     	; 0x14 <__ctors_end>
   2:	4f c0       	rjmp	.+158    	; 0xa2 <__vector_1>
   4:	15 c0       	rjmp	.+42     	; 0x30 <__bad_interrupt>
   6:	14 c0       	rjmp	.+40     	; 0x30 <__bad_interrupt>
   8:	13 c0       	rjmp	.+38     	; 0x30 <__bad_interrupt>
   a:	12 c0       	rjmp	.+36     	; 0x30 <__bad_interrupt>
   c:	11 c0       	rjmp	.+34     	; 0x30 <__bad_interrupt>
   e:	10 c0       	rjmp	.+32     	; 0x30 <__bad_interrupt>
  10:	0f c0       	rjmp	.+30     	; 0x30 <__bad_interrupt>
  12:	0e c0       	rjmp	.+28     	; 0x30 <__bad_interrupt>

00000014 <__ctors_end>:
  14:	11 24       	eor	r1, r1
  16:	1f be       	out	0x3f, r1	; 63
  18:	cf e9       	ldi	r28, 0x9F	; 159
  1a:	cd bf       	out	0x3d, r28	; 61

0000001c <__do_clear_bss>:
  1c:	10 e0       	ldi	r17, 0x00	; 0
  1e:	a0 e6       	ldi	r26, 0x60	; 96
  20:	b0 e0       	ldi	r27, 0x00	; 0
  22:	01 c0       	rjmp	.+2      	; 0x26 <.do_clear_bss_start>

00000024 <.do_clear_bss_loop>:
  24:	1d 92       	st	X+, r1

00000026 <.do_clear_bss_start>:
  26:	a2 36       	cpi	r26, 0x62	; 98
  28:	b1 07       	cpc	r27, r17
  2a:	e1 f7       	brne	.-8      	; 0x24 <.do_clear_bss_loop>
  2c:	96 d0       	rcall	.+300    	; 0x15a <main>
  2e:	cc c0       	rjmp	.+408    	; 0x1c8 <_exit>

00000030 <__bad_interrupt>:
  30:	e7 cf       	rjmp	.-50     	; 0x0 <__vectors>

00000032 <spi_init>:
  32:	8e e0       	ldi	r24, 0x0E	; 14
  34:	87 bb       	out	0x17, r24	; 23
  36:	c3 9a       	sbi	0x18, 3	; 24
  38:	c1 9a       	sbi	0x18, 1	; 24
  3a:	08 95       	ret

0000003c <pwm_init>:
  3c:	83 b7       	in	r24, 0x33	; 51
  3e:	81 60       	ori	r24, 0x01	; 1
  40:	83 bf       	out	0x33, r24	; 51
  42:	8f b5       	in	r24, 0x2f	; 47
  44:	83 60       	ori	r24, 0x03	; 3
  46:	8f bd       	out	0x2f, r24	; 47
  48:	8f b5       	in	r24, 0x2f	; 47
  4a:	80 68       	ori	r24, 0x80	; 128
  4c:	8f bd       	out	0x2f, r24	; 47
  4e:	b8 9a       	sbi	0x17, 0	; 23
  50:	08 95       	ret

00000052 <uart_send_byte>:
  52:	99 b7       	in	r25, 0x39	; 57
  54:	9b 7f       	andi	r25, 0xFB	; 251
  56:	99 bf       	out	0x39, r25	; 57
  58:	98 b7       	in	r25, 0x38	; 56
  5a:	94 60       	ori	r25, 0x04	; 4
  5c:	98 bf       	out	0x38, r25	; 56
  5e:	12 be       	out	0x32, r1	; 50
  60:	99 b7       	in	r25, 0x39	; 57
  62:	91 60       	ori	r25, 0x01	; 1
  64:	99 bf       	out	0x39, r25	; 57
  66:	c0 98       	cbi	0x18, 0	; 24
  68:	08 b6       	in	r0, 0x38	; 56
  6a:	02 fe       	sbrs	r0, 2
  6c:	fd cf       	rjmp	.-6      	; 0x68 <uart_send_byte+0x16>
  6e:	98 b7       	in	r25, 0x38	; 56
  70:	94 60       	ori	r25, 0x04	; 4
  72:	98 bf       	out	0x38, r25	; 56
  74:	98 e0       	ldi	r25, 0x08	; 8
  76:	80 ff       	sbrs	r24, 0
  78:	02 c0       	rjmp	.+4      	; 0x7e <uart_send_byte+0x2c>
  7a:	c0 9a       	sbi	0x18, 0	; 24
  7c:	01 c0       	rjmp	.+2      	; 0x80 <uart_send_byte+0x2e>
  7e:	c0 98       	cbi	0x18, 0	; 24
  80:	86 95       	lsr	r24
  82:	08 b6       	in	r0, 0x38	; 56
  84:	02 fe       	sbrs	r0, 2
  86:	fd cf       	rjmp	.-6      	; 0x82 <uart_send_byte+0x30>
  88:	28 b7       	in	r18, 0x38	; 56
  8a:	24 60       	ori	r18, 0x04	; 4
  8c:	28 bf       	out	0x38, r18	; 56
  8e:	91 50       	subi	r25, 0x01	; 1
  90:	91 f7       	brne	.-28     	; 0x76 <uart_send_byte+0x24>
  92:	c0 9a       	sbi	0x18, 0	; 24
  94:	08 b6       	in	r0, 0x38	; 56
  96:	02 fe       	sbrs	r0, 2
  98:	fd cf       	rjmp	.-6      	; 0x94 <uart_send_byte+0x42>
  9a:	88 b7       	in	r24, 0x38	; 56
  9c:	84 60       	ori	r24, 0x04	; 4
  9e:	88 bf       	out	0x38, r24	; 56
  a0:	08 95       	ret

000000a2 <__vector_1>:
  a2:	1f 92       	push	r1
  a4:	0f 92       	push	r0
  a6:	0f b6       	in	r0, 0x3f	; 63
  a8:	0f 92       	push	r0
  aa:	11 24       	eor	r1, r1
  ac:	2f 93       	push	r18
  ae:	8f 93       	push	r24
  b0:	9f 93       	push	r25
  b2:	10 92 60 00 	sts	0x0060, r1
  b6:	89 b7       	in	r24, 0x39	; 57
  b8:	8b 7f       	andi	r24, 0xFB	; 251
  ba:	89 bf       	out	0x39, r24	; 57
  bc:	88 b7       	in	r24, 0x38	; 56
  be:	84 60       	ori	r24, 0x04	; 4
  c0:	88 bf       	out	0x38, r24	; 56
  c2:	12 be       	out	0x32, r1	; 50
  c4:	89 b7       	in	r24, 0x39	; 57
  c6:	84 60       	ori	r24, 0x04	; 4
  c8:	89 bf       	out	0x39, r24	; 57
  ca:	88 e0       	ldi	r24, 0x08	; 8
  cc:	90 e0       	ldi	r25, 0x00	; 0
  ce:	08 b6       	in	r0, 0x38	; 56
  d0:	02 fe       	sbrs	r0, 2
  d2:	fd cf       	rjmp	.-6      	; 0xce <__vector_1+0x2c>
  d4:	28 b7       	in	r18, 0x38	; 56
  d6:	24 60       	ori	r18, 0x04	; 4
  d8:	28 bf       	out	0x38, r18	; 56
  da:	20 91 60 00 	lds	r18, 0x0060
  de:	22 0f       	add	r18, r18
  e0:	20 93 60 00 	sts	0x0060, r18
  e4:	b1 9b       	sbis	0x16, 1	; 22
  e6:	03 c0       	rjmp	.+6      	; 0xee <__vector_1+0x4c>
  e8:	21 60       	ori	r18, 0x01	; 1
  ea:	20 93 60 00 	sts	0x0060, r18
  ee:	01 97       	sbiw	r24, 0x01	; 1
  f0:	00 97       	sbiw	r24, 0x00	; 0
  f2:	69 f7       	brne	.-38     	; 0xce <__vector_1+0x2c>
  f4:	81 e0       	ldi	r24, 0x01	; 1
  f6:	80 93 61 00 	sts	0x0061, r24
  fa:	08 b6       	in	r0, 0x38	; 56
  fc:	02 fe       	sbrs	r0, 2
  fe:	fd cf       	rjmp	.-6      	; 0xfa <__vector_1+0x58>
 100:	88 b7       	in	r24, 0x38	; 56
 102:	84 60       	ori	r24, 0x04	; 4
 104:	88 bf       	out	0x38, r24	; 56
 106:	8a b7       	in	r24, 0x3a	; 58
 108:	80 64       	ori	r24, 0x40	; 64
 10a:	8a bf       	out	0x3a, r24	; 58
 10c:	9f 91       	pop	r25
 10e:	8f 91       	pop	r24
 110:	2f 91       	pop	r18
 112:	0f 90       	pop	r0
 114:	0f be       	out	0x3f, r0	; 63
 116:	0f 90       	pop	r0
 118:	1f 90       	pop	r1
 11a:	18 95       	reti

0000011c <uart_rx_int_init>:
 11c:	b9 98       	cbi	0x17, 1	; 23
 11e:	c1 9a       	sbi	0x18, 1	; 24
 120:	82 e0       	ldi	r24, 0x02	; 2
 122:	85 bf       	out	0x35, r24	; 53
 124:	8b b7       	in	r24, 0x3b	; 59
 126:	80 64       	ori	r24, 0x40	; 64
 128:	8b bf       	out	0x3b, r24	; 59
 12a:	08 95       	ret

0000012c <uart_tx_init>:
 12c:	82 e0       	ldi	r24, 0x02	; 2
 12e:	8f bd       	out	0x2f, r24	; 47
 130:	81 e0       	ldi	r24, 0x01	; 1
 132:	83 bf       	out	0x33, r24	; 51
 134:	3e 9a       	sbi	0x07, 6	; 7
 136:	b8 9a       	sbi	0x17, 0	; 23
 138:	8b e4       	ldi	r24, 0x4B	; 75
 13a:	86 bf       	out	0x36, r24	; 54
 13c:	08 95       	ret

0000013e <spi_transmit_receive>:
 13e:	98 e0       	ldi	r25, 0x08	; 8
 140:	87 ff       	sbrs	r24, 7
 142:	02 c0       	rjmp	.+4      	; 0x148 <spi_transmit_receive+0xa>
 144:	c2 9a       	sbi	0x18, 2	; 24
 146:	01 c0       	rjmp	.+2      	; 0x14a <spi_transmit_receive+0xc>
 148:	c2 98       	cbi	0x18, 2	; 24
 14a:	c1 98       	cbi	0x18, 1	; 24
 14c:	c1 9a       	sbi	0x18, 1	; 24
 14e:	88 0f       	add	r24, r24
 150:	b4 99       	sbic	0x16, 4	; 22
 152:	81 60       	ori	r24, 0x01	; 1
 154:	91 50       	subi	r25, 0x01	; 1
 156:	a1 f7       	brne	.-24     	; 0x140 <spi_transmit_receive+0x2>
 158:	08 95       	ret

0000015a <main>:
 15a:	80 e0       	ldi	r24, 0x00	; 0
 15c:	90 e0       	ldi	r25, 0x00	; 0
 15e:	20 e8       	ldi	r18, 0x80	; 128
 160:	0f b6       	in	r0, 0x3f	; 63
 162:	f8 94       	cli
 164:	26 bd       	out	0x26, r18	; 38
 166:	86 bd       	out	0x26, r24	; 38
 168:	0f be       	out	0x3f, r0	; 63
 16a:	bb 9a       	sbi	0x17, 3	; 23
 16c:	c3 9a       	sbi	0x18, 3	; 24
 16e:	b8 9a       	sbi	0x17, 0	; 23
 170:	c0 9a       	sbi	0x18, 0	; 24
 172:	b9 98       	cbi	0x17, 1	; 23
 174:	c1 9a       	sbi	0x18, 1	; 24
 176:	da df       	rcall	.-76     	; 0x12c <uart_tx_init>
 178:	d1 df       	rcall	.-94     	; 0x11c <uart_rx_int_init>
 17a:	78 94       	sei
 17c:	80 91 61 00 	lds	r24, 0x0061
 180:	88 23       	and	r24, r24
 182:	b1 f0       	breq	.+44     	; 0x1b0 <main+0x56>
 184:	2f ef       	ldi	r18, 0xFF	; 255
 186:	8d ee       	ldi	r24, 0xED	; 237
 188:	92 e0       	ldi	r25, 0x02	; 2
 18a:	21 50       	subi	r18, 0x01	; 1
 18c:	80 40       	sbci	r24, 0x00	; 0
 18e:	90 40       	sbci	r25, 0x00	; 0
 190:	e1 f7       	brne	.-8      	; 0x18a <main+0x30>
 192:	00 c0       	rjmp	.+0      	; 0x194 <main+0x3a>
 194:	00 00       	nop
 196:	81 e4       	ldi	r24, 0x41	; 65
 198:	5c df       	rcall	.-328    	; 0x52 <uart_send_byte>
 19a:	10 92 61 00 	sts	0x0061, r1
 19e:	2f ef       	ldi	r18, 0xFF	; 255
 1a0:	8d ee       	ldi	r24, 0xED	; 237
 1a2:	92 e0       	ldi	r25, 0x02	; 2
 1a4:	21 50       	subi	r18, 0x01	; 1
 1a6:	80 40       	sbci	r24, 0x00	; 0
 1a8:	90 40       	sbci	r25, 0x00	; 0
 1aa:	e1 f7       	brne	.-8      	; 0x1a4 <main+0x4a>
 1ac:	00 c0       	rjmp	.+0      	; 0x1ae <main+0x54>
 1ae:	00 00       	nop
 1b0:	2f ef       	ldi	r18, 0xFF	; 255
 1b2:	8d ee       	ldi	r24, 0xED	; 237
 1b4:	92 e0       	ldi	r25, 0x02	; 2
 1b6:	21 50       	subi	r18, 0x01	; 1
 1b8:	80 40       	sbci	r24, 0x00	; 0
 1ba:	90 40       	sbci	r25, 0x00	; 0
 1bc:	e1 f7       	brne	.-8      	; 0x1b6 <main+0x5c>
 1be:	00 c0       	rjmp	.+0      	; 0x1c0 <main+0x66>
 1c0:	00 00       	nop
 1c2:	82 e4       	ldi	r24, 0x42	; 66
 1c4:	46 df       	rcall	.-372    	; 0x52 <uart_send_byte>
 1c6:	da cf       	rjmp	.-76     	; 0x17c <main+0x22>

000001c8 <_exit>:
 1c8:	f8 94       	cli

000001ca <__stop_program>:
 1ca:	ff cf       	rjmp	.-2      	; 0x1ca <__stop_program>

Disassembly of section .bss:

00800060 <__bss_start>:
	...

00800061 <uart_rx_flag>:
	...

Disassembly of section .stab:

00000000 <_end-0x800062>:
   0:	01 00       	.word	0x0001	; ????
   2:	00 00       	nop
   4:	00 00       	nop
   6:	8e 00       	.word	0x008e	; ????
   8:	54 00       	.word	0x0054	; ????
   a:	00 00       	nop
   c:	01 00       	.word	0x0001	; ????
   e:	00 00       	nop
  10:	64 00       	.word	0x0064	; ????
  12:	00 00       	nop
  14:	30 00       	.word	0x0030	; ????
  16:	00 00       	nop
  18:	11 00       	.word	0x0011	; ????
  1a:	00 00       	nop
  1c:	80 00       	.word	0x0080	; ????
  1e:	00 00       	nop
  20:	00 00       	nop
  22:	00 00       	nop
  24:	1b 00       	.word	0x001b	; ????
  26:	00 00       	nop
  28:	24 00       	.word	0x0024	; ????
  2a:	33 00       	.word	0x0033	; ????
  2c:	00 00       	nop
  2e:	00 00       	nop
  30:	28 00       	.word	0x0028	; ????
  32:	00 00       	nop
  34:	84 00       	.word	0x0084	; ????
	...
  3e:	00 00       	nop
  40:	44 00       	.word	0x0044	; ????
  42:	34 00       	.word	0x0034	; ????
	...
  4c:	44 00       	.word	0x0044	; ????
  4e:	35 00       	.word	0x0035	; ????
  50:	02 00       	.word	0x0002	; ????
  52:	00 00       	nop
  54:	00 00       	nop
  56:	00 00       	nop
  58:	44 00       	.word	0x0044	; ????
  5a:	36 00       	.word	0x0036	; ????
  5c:	04 00       	.word	0x0004	; ????
  5e:	00 00       	nop
  60:	00 00       	nop
  62:	00 00       	nop
  64:	44 00       	.word	0x0044	; ????
  66:	37 00       	.word	0x0037	; ????
  68:	06 00       	.word	0x0006	; ????
  6a:	00 00       	nop
  6c:	00 00       	nop
  6e:	00 00       	nop
  70:	44 00       	.word	0x0044	; ????
  72:	38 00       	.word	0x0038	; ????
  74:	08 00       	.word	0x0008	; ????
  76:	00 00       	nop
  78:	00 00       	nop
  7a:	00 00       	nop
  7c:	44 00       	.word	0x0044	; ????
  7e:	39 00       	.word	0x0039	; ????
  80:	0a 00       	.word	0x000a	; ????
  82:	00 00       	nop
  84:	00 00       	nop
  86:	00 00       	nop
  88:	44 00       	.word	0x0044	; ????
  8a:	3a 00       	.word	0x003a	; ????
  8c:	0c 00       	.word	0x000c	; ????
  8e:	00 00       	nop
  90:	00 00       	nop
  92:	00 00       	nop
  94:	44 00       	.word	0x0044	; ????
  96:	3b 00       	.word	0x003b	; ????
  98:	0e 00       	.word	0x000e	; ????
  9a:	00 00       	nop
  9c:	00 00       	nop
  9e:	00 00       	nop
  a0:	44 00       	.word	0x0044	; ????
  a2:	3c 00       	.word	0x003c	; ????
  a4:	10 00       	.word	0x0010	; ????
  a6:	00 00       	nop
  a8:	00 00       	nop
  aa:	00 00       	nop
  ac:	44 00       	.word	0x0044	; ????
  ae:	3d 00       	.word	0x003d	; ????
  b0:	12 00       	.word	0x0012	; ????
  b2:	00 00       	nop
  b4:	00 00       	nop
  b6:	00 00       	nop
  b8:	44 00       	.word	0x0044	; ????
  ba:	3e 00       	.word	0x003e	; ????
  bc:	14 00       	.word	0x0014	; ????
  be:	00 00       	nop
  c0:	00 00       	nop
  c2:	00 00       	nop
  c4:	44 00       	.word	0x0044	; ????
  c6:	3f 00       	.word	0x003f	; ????
  c8:	14 00       	.word	0x0014	; ????
  ca:	00 00       	nop
  cc:	00 00       	nop
  ce:	00 00       	nop
  d0:	44 00       	.word	0x0044	; ????
  d2:	40 00       	.word	0x0040	; ????
  d4:	14 00       	.word	0x0014	; ????
  d6:	00 00       	nop
  d8:	00 00       	nop
  da:	00 00       	nop
  dc:	44 00       	.word	0x0044	; ????
  de:	41 00       	.word	0x0041	; ????
  e0:	14 00       	.word	0x0014	; ????
  e2:	00 00       	nop
  e4:	00 00       	nop
  e6:	00 00       	nop
  e8:	44 00       	.word	0x0044	; ????
  ea:	42 00       	.word	0x0042	; ????
  ec:	14 00       	.word	0x0014	; ????
  ee:	00 00       	nop
  f0:	00 00       	nop
  f2:	00 00       	nop
  f4:	44 00       	.word	0x0044	; ????
  f6:	43 00       	.word	0x0043	; ????
  f8:	14 00       	.word	0x0014	; ????
  fa:	00 00       	nop
  fc:	00 00       	nop
  fe:	00 00       	nop
 100:	44 00       	.word	0x0044	; ????
 102:	44 00       	.word	0x0044	; ????
 104:	14 00       	.word	0x0014	; ????
 106:	00 00       	nop
 108:	00 00       	nop
 10a:	00 00       	nop
 10c:	44 00       	.word	0x0044	; ????
 10e:	45 00       	.word	0x0045	; ????
 110:	14 00       	.word	0x0014	; ????
 112:	00 00       	nop
 114:	00 00       	nop
 116:	00 00       	nop
 118:	44 00       	.word	0x0044	; ????
 11a:	46 00       	.word	0x0046	; ????
 11c:	14 00       	.word	0x0014	; ????
 11e:	00 00       	nop
 120:	00 00       	nop
 122:	00 00       	nop
 124:	44 00       	.word	0x0044	; ????
 126:	47 00       	.word	0x0047	; ????
 128:	14 00       	.word	0x0014	; ????
 12a:	00 00       	nop
 12c:	00 00       	nop
 12e:	00 00       	nop
 130:	44 00       	.word	0x0044	; ????
 132:	48 00       	.word	0x0048	; ????
 134:	14 00       	.word	0x0014	; ????
 136:	00 00       	nop
 138:	00 00       	nop
 13a:	00 00       	nop
 13c:	44 00       	.word	0x0044	; ????
 13e:	49 00       	.word	0x0049	; ????
 140:	14 00       	.word	0x0014	; ????
 142:	00 00       	nop
 144:	00 00       	nop
 146:	00 00       	nop
 148:	44 00       	.word	0x0044	; ????
 14a:	4a 00       	.word	0x004a	; ????
 14c:	14 00       	.word	0x0014	; ????
 14e:	00 00       	nop
 150:	00 00       	nop
 152:	00 00       	nop
 154:	44 00       	.word	0x0044	; ????
 156:	4b 00       	.word	0x004b	; ????
 158:	14 00       	.word	0x0014	; ????
 15a:	00 00       	nop
 15c:	00 00       	nop
 15e:	00 00       	nop
 160:	44 00       	.word	0x0044	; ????
 162:	4c 00       	.word	0x004c	; ????
 164:	14 00       	.word	0x0014	; ????
 166:	00 00       	nop
 168:	00 00       	nop
 16a:	00 00       	nop
 16c:	44 00       	.word	0x0044	; ????
 16e:	4d 00       	.word	0x004d	; ????
 170:	14 00       	.word	0x0014	; ????
 172:	00 00       	nop
 174:	00 00       	nop
 176:	00 00       	nop
 178:	44 00       	.word	0x0044	; ????
 17a:	4e 00       	.word	0x004e	; ????
 17c:	14 00       	.word	0x0014	; ????
 17e:	00 00       	nop
 180:	00 00       	nop
 182:	00 00       	nop
 184:	44 00       	.word	0x0044	; ????
 186:	4f 00       	.word	0x004f	; ????
 188:	14 00       	.word	0x0014	; ????
 18a:	00 00       	nop
 18c:	00 00       	nop
 18e:	00 00       	nop
 190:	44 00       	.word	0x0044	; ????
 192:	50 00       	.word	0x0050	; ????
 194:	14 00       	.word	0x0014	; ????
 196:	00 00       	nop
 198:	00 00       	nop
 19a:	00 00       	nop
 19c:	44 00       	.word	0x0044	; ????
 19e:	51 00       	.word	0x0051	; ????
 1a0:	14 00       	.word	0x0014	; ????
 1a2:	00 00       	nop
 1a4:	00 00       	nop
 1a6:	00 00       	nop
 1a8:	44 00       	.word	0x0044	; ????
 1aa:	52 00       	.word	0x0052	; ????
 1ac:	14 00       	.word	0x0014	; ????
 1ae:	00 00       	nop
 1b0:	00 00       	nop
 1b2:	00 00       	nop
 1b4:	44 00       	.word	0x0044	; ????
 1b6:	53 00       	.word	0x0053	; ????
 1b8:	14 00       	.word	0x0014	; ????
 1ba:	00 00       	nop
 1bc:	00 00       	nop
 1be:	00 00       	nop
 1c0:	44 00       	.word	0x0044	; ????
 1c2:	54 00       	.word	0x0054	; ????
 1c4:	14 00       	.word	0x0014	; ????
 1c6:	00 00       	nop
 1c8:	00 00       	nop
 1ca:	00 00       	nop
 1cc:	44 00       	.word	0x0044	; ????
 1ce:	55 00       	.word	0x0055	; ????
 1d0:	14 00       	.word	0x0014	; ????
 1d2:	00 00       	nop
 1d4:	00 00       	nop
 1d6:	00 00       	nop
 1d8:	44 00       	.word	0x0044	; ????
 1da:	56 00       	.word	0x0056	; ????
 1dc:	14 00       	.word	0x0014	; ????
 1de:	00 00       	nop
 1e0:	00 00       	nop
 1e2:	00 00       	nop
 1e4:	44 00       	.word	0x0044	; ????
 1e6:	57 00       	.word	0x0057	; ????
 1e8:	14 00       	.word	0x0014	; ????
 1ea:	00 00       	nop
 1ec:	00 00       	nop
 1ee:	00 00       	nop
 1f0:	44 00       	.word	0x0044	; ????
 1f2:	58 00       	.word	0x0058	; ????
 1f4:	14 00       	.word	0x0014	; ????
 1f6:	00 00       	nop
 1f8:	00 00       	nop
 1fa:	00 00       	nop
 1fc:	44 00       	.word	0x0044	; ????
 1fe:	59 00       	.word	0x0059	; ????
 200:	14 00       	.word	0x0014	; ????
 202:	00 00       	nop
 204:	00 00       	nop
 206:	00 00       	nop
 208:	44 00       	.word	0x0044	; ????
 20a:	5a 00       	.word	0x005a	; ????
 20c:	14 00       	.word	0x0014	; ????
 20e:	00 00       	nop
 210:	00 00       	nop
 212:	00 00       	nop
 214:	44 00       	.word	0x0044	; ????
 216:	5b 00       	.word	0x005b	; ????
 218:	14 00       	.word	0x0014	; ????
 21a:	00 00       	nop
 21c:	00 00       	nop
 21e:	00 00       	nop
 220:	44 00       	.word	0x0044	; ????
 222:	5c 00       	.word	0x005c	; ????
 224:	14 00       	.word	0x0014	; ????
 226:	00 00       	nop
 228:	00 00       	nop
 22a:	00 00       	nop
 22c:	44 00       	.word	0x0044	; ????
 22e:	5d 00       	.word	0x005d	; ????
 230:	14 00       	.word	0x0014	; ????
 232:	00 00       	nop
 234:	00 00       	nop
 236:	00 00       	nop
 238:	44 00       	.word	0x0044	; ????
 23a:	5e 00       	.word	0x005e	; ????
 23c:	14 00       	.word	0x0014	; ????
 23e:	00 00       	nop
 240:	00 00       	nop
 242:	00 00       	nop
 244:	44 00       	.word	0x0044	; ????
 246:	5f 00       	.word	0x005f	; ????
 248:	14 00       	.word	0x0014	; ????
 24a:	00 00       	nop
 24c:	00 00       	nop
 24e:	00 00       	nop
 250:	44 00       	.word	0x0044	; ????
 252:	60 00       	.word	0x0060	; ????
 254:	14 00       	.word	0x0014	; ????
 256:	00 00       	nop
 258:	00 00       	nop
 25a:	00 00       	nop
 25c:	44 00       	.word	0x0044	; ????
 25e:	61 00       	.word	0x0061	; ????
 260:	14 00       	.word	0x0014	; ????
 262:	00 00       	nop
 264:	00 00       	nop
 266:	00 00       	nop
 268:	44 00       	.word	0x0044	; ????
 26a:	62 00       	.word	0x0062	; ????
 26c:	14 00       	.word	0x0014	; ????
 26e:	00 00       	nop
 270:	00 00       	nop
 272:	00 00       	nop
 274:	44 00       	.word	0x0044	; ????
 276:	63 00       	.word	0x0063	; ????
 278:	14 00       	.word	0x0014	; ????
 27a:	00 00       	nop
 27c:	00 00       	nop
 27e:	00 00       	nop
 280:	44 00       	.word	0x0044	; ????
 282:	64 00       	.word	0x0064	; ????
 284:	14 00       	.word	0x0014	; ????
 286:	00 00       	nop
 288:	00 00       	nop
 28a:	00 00       	nop
 28c:	44 00       	.word	0x0044	; ????
 28e:	65 00       	.word	0x0065	; ????
 290:	14 00       	.word	0x0014	; ????
 292:	00 00       	nop
 294:	00 00       	nop
 296:	00 00       	nop
 298:	44 00       	.word	0x0044	; ????
 29a:	66 00       	.word	0x0066	; ????
 29c:	14 00       	.word	0x0014	; ????
 29e:	00 00       	nop
 2a0:	00 00       	nop
 2a2:	00 00       	nop
 2a4:	44 00       	.word	0x0044	; ????
 2a6:	67 00       	.word	0x0067	; ????
 2a8:	14 00       	.word	0x0014	; ????
 2aa:	00 00       	nop
 2ac:	00 00       	nop
 2ae:	00 00       	nop
 2b0:	44 00       	.word	0x0044	; ????
 2b2:	68 00       	.word	0x0068	; ????
 2b4:	14 00       	.word	0x0014	; ????
 2b6:	00 00       	nop
 2b8:	00 00       	nop
 2ba:	00 00       	nop
 2bc:	44 00       	.word	0x0044	; ????
 2be:	69 00       	.word	0x0069	; ????
 2c0:	14 00       	.word	0x0014	; ????
 2c2:	00 00       	nop
 2c4:	00 00       	nop
 2c6:	00 00       	nop
 2c8:	44 00       	.word	0x0044	; ????
 2ca:	6a 00       	.word	0x006a	; ????
 2cc:	14 00       	.word	0x0014	; ????
 2ce:	00 00       	nop
 2d0:	00 00       	nop
 2d2:	00 00       	nop
 2d4:	44 00       	.word	0x0044	; ????
 2d6:	6b 00       	.word	0x006b	; ????
 2d8:	14 00       	.word	0x0014	; ????
 2da:	00 00       	nop
 2dc:	00 00       	nop
 2de:	00 00       	nop
 2e0:	44 00       	.word	0x0044	; ????
 2e2:	6c 00       	.word	0x006c	; ????
 2e4:	14 00       	.word	0x0014	; ????
 2e6:	00 00       	nop
 2e8:	00 00       	nop
 2ea:	00 00       	nop
 2ec:	44 00       	.word	0x0044	; ????
 2ee:	6d 00       	.word	0x006d	; ????
 2f0:	14 00       	.word	0x0014	; ????
 2f2:	00 00       	nop
 2f4:	00 00       	nop
 2f6:	00 00       	nop
 2f8:	44 00       	.word	0x0044	; ????
 2fa:	6e 00       	.word	0x006e	; ????
 2fc:	14 00       	.word	0x0014	; ????
 2fe:	00 00       	nop
 300:	00 00       	nop
 302:	00 00       	nop
 304:	44 00       	.word	0x0044	; ????
 306:	6f 00       	.word	0x006f	; ????
 308:	14 00       	.word	0x0014	; ????
 30a:	00 00       	nop
 30c:	00 00       	nop
 30e:	00 00       	nop
 310:	44 00       	.word	0x0044	; ????
 312:	70 00       	.word	0x0070	; ????
 314:	14 00       	.word	0x0014	; ????
 316:	00 00       	nop
 318:	00 00       	nop
 31a:	00 00       	nop
 31c:	44 00       	.word	0x0044	; ????
 31e:	71 00       	.word	0x0071	; ????
 320:	14 00       	.word	0x0014	; ????
 322:	00 00       	nop
 324:	00 00       	nop
 326:	00 00       	nop
 328:	44 00       	.word	0x0044	; ????
 32a:	72 00       	.word	0x0072	; ????
 32c:	14 00       	.word	0x0014	; ????
 32e:	00 00       	nop
 330:	00 00       	nop
 332:	00 00       	nop
 334:	44 00       	.word	0x0044	; ????
 336:	73 00       	.word	0x0073	; ????
 338:	14 00       	.word	0x0014	; ????
 33a:	00 00       	nop
 33c:	00 00       	nop
 33e:	00 00       	nop
 340:	44 00       	.word	0x0044	; ????
 342:	74 00       	.word	0x0074	; ????
 344:	14 00       	.word	0x0014	; ????
 346:	00 00       	nop
 348:	00 00       	nop
 34a:	00 00       	nop
 34c:	44 00       	.word	0x0044	; ????
 34e:	75 00       	.word	0x0075	; ????
 350:	14 00       	.word	0x0014	; ????
 352:	00 00       	nop
 354:	00 00       	nop
 356:	00 00       	nop
 358:	44 00       	.word	0x0044	; ????
 35a:	76 00       	.word	0x0076	; ????
 35c:	14 00       	.word	0x0014	; ????
 35e:	00 00       	nop
 360:	00 00       	nop
 362:	00 00       	nop
 364:	44 00       	.word	0x0044	; ????
 366:	77 00       	.word	0x0077	; ????
 368:	14 00       	.word	0x0014	; ????
 36a:	00 00       	nop
 36c:	00 00       	nop
 36e:	00 00       	nop
 370:	44 00       	.word	0x0044	; ????
 372:	78 00       	.word	0x0078	; ????
 374:	14 00       	.word	0x0014	; ????
 376:	00 00       	nop
 378:	00 00       	nop
 37a:	00 00       	nop
 37c:	44 00       	.word	0x0044	; ????
 37e:	79 00       	.word	0x0079	; ????
 380:	14 00       	.word	0x0014	; ????
 382:	00 00       	nop
 384:	00 00       	nop
 386:	00 00       	nop
 388:	44 00       	.word	0x0044	; ????
 38a:	7a 00       	.word	0x007a	; ????
 38c:	14 00       	.word	0x0014	; ????
 38e:	00 00       	nop
 390:	00 00       	nop
 392:	00 00       	nop
 394:	44 00       	.word	0x0044	; ????
 396:	7b 00       	.word	0x007b	; ????
 398:	14 00       	.word	0x0014	; ????
 39a:	00 00       	nop
 39c:	00 00       	nop
 39e:	00 00       	nop
 3a0:	44 00       	.word	0x0044	; ????
 3a2:	7c 00       	.word	0x007c	; ????
 3a4:	14 00       	.word	0x0014	; ????
 3a6:	00 00       	nop
 3a8:	00 00       	nop
 3aa:	00 00       	nop
 3ac:	44 00       	.word	0x0044	; ????
 3ae:	7d 00       	.word	0x007d	; ????
 3b0:	14 00       	.word	0x0014	; ????
 3b2:	00 00       	nop
 3b4:	00 00       	nop
 3b6:	00 00       	nop
 3b8:	44 00       	.word	0x0044	; ????
 3ba:	7e 00       	.word	0x007e	; ????
 3bc:	14 00       	.word	0x0014	; ????
 3be:	00 00       	nop
 3c0:	00 00       	nop
 3c2:	00 00       	nop
 3c4:	44 00       	.word	0x0044	; ????
 3c6:	7f 00       	.word	0x007f	; ????
 3c8:	14 00       	.word	0x0014	; ????
 3ca:	00 00       	nop
 3cc:	00 00       	nop
 3ce:	00 00       	nop
 3d0:	44 00       	.word	0x0044	; ????
 3d2:	80 00       	.word	0x0080	; ????
 3d4:	14 00       	.word	0x0014	; ????
 3d6:	00 00       	nop
 3d8:	00 00       	nop
 3da:	00 00       	nop
 3dc:	44 00       	.word	0x0044	; ????
 3de:	81 00       	.word	0x0081	; ????
 3e0:	14 00       	.word	0x0014	; ????
 3e2:	00 00       	nop
 3e4:	00 00       	nop
 3e6:	00 00       	nop
 3e8:	44 00       	.word	0x0044	; ????
 3ea:	82 00       	.word	0x0082	; ????
 3ec:	14 00       	.word	0x0014	; ????
 3ee:	00 00       	nop
 3f0:	00 00       	nop
 3f2:	00 00       	nop
 3f4:	44 00       	.word	0x0044	; ????
 3f6:	83 00       	.word	0x0083	; ????
 3f8:	14 00       	.word	0x0014	; ????
 3fa:	00 00       	nop
 3fc:	00 00       	nop
 3fe:	00 00       	nop
 400:	44 00       	.word	0x0044	; ????
 402:	84 00       	.word	0x0084	; ????
 404:	14 00       	.word	0x0014	; ????
 406:	00 00       	nop
 408:	00 00       	nop
 40a:	00 00       	nop
 40c:	44 00       	.word	0x0044	; ????
 40e:	85 00       	.word	0x0085	; ????
 410:	14 00       	.word	0x0014	; ????
 412:	00 00       	nop
 414:	00 00       	nop
 416:	00 00       	nop
 418:	44 00       	.word	0x0044	; ????
 41a:	86 00       	.word	0x0086	; ????
 41c:	14 00       	.word	0x0014	; ????
 41e:	00 00       	nop
 420:	00 00       	nop
 422:	00 00       	nop
 424:	44 00       	.word	0x0044	; ????
 426:	87 00       	.word	0x0087	; ????
 428:	14 00       	.word	0x0014	; ????
 42a:	00 00       	nop
 42c:	00 00       	nop
 42e:	00 00       	nop
 430:	44 00       	.word	0x0044	; ????
 432:	88 00       	.word	0x0088	; ????
 434:	14 00       	.word	0x0014	; ????
 436:	00 00       	nop
 438:	00 00       	nop
 43a:	00 00       	nop
 43c:	44 00       	.word	0x0044	; ????
 43e:	89 00       	.word	0x0089	; ????
 440:	14 00       	.word	0x0014	; ????
 442:	00 00       	nop
 444:	00 00       	nop
 446:	00 00       	nop
 448:	44 00       	.word	0x0044	; ????
 44a:	8a 00       	.word	0x008a	; ????
 44c:	14 00       	.word	0x0014	; ????
 44e:	00 00       	nop
 450:	00 00       	nop
 452:	00 00       	nop
 454:	44 00       	.word	0x0044	; ????
 456:	8b 00       	.word	0x008b	; ????
 458:	14 00       	.word	0x0014	; ????
 45a:	00 00       	nop
 45c:	00 00       	nop
 45e:	00 00       	nop
 460:	44 00       	.word	0x0044	; ????
 462:	8c 00       	.word	0x008c	; ????
 464:	14 00       	.word	0x0014	; ????
 466:	00 00       	nop
 468:	00 00       	nop
 46a:	00 00       	nop
 46c:	44 00       	.word	0x0044	; ????
 46e:	8d 00       	.word	0x008d	; ????
 470:	14 00       	.word	0x0014	; ????
 472:	00 00       	nop
 474:	00 00       	nop
 476:	00 00       	nop
 478:	44 00       	.word	0x0044	; ????
 47a:	8e 00       	.word	0x008e	; ????
 47c:	14 00       	.word	0x0014	; ????
 47e:	00 00       	nop
 480:	00 00       	nop
 482:	00 00       	nop
 484:	44 00       	.word	0x0044	; ????
 486:	8f 00       	.word	0x008f	; ????
 488:	14 00       	.word	0x0014	; ????
 48a:	00 00       	nop
 48c:	00 00       	nop
 48e:	00 00       	nop
 490:	44 00       	.word	0x0044	; ????
 492:	90 00       	.word	0x0090	; ????
 494:	14 00       	.word	0x0014	; ????
 496:	00 00       	nop
 498:	00 00       	nop
 49a:	00 00       	nop
 49c:	44 00       	.word	0x0044	; ????
 49e:	91 00       	.word	0x0091	; ????
 4a0:	14 00       	.word	0x0014	; ????
 4a2:	00 00       	nop
 4a4:	00 00       	nop
 4a6:	00 00       	nop
 4a8:	44 00       	.word	0x0044	; ????
 4aa:	92 00       	.word	0x0092	; ????
 4ac:	14 00       	.word	0x0014	; ????
 4ae:	00 00       	nop
 4b0:	00 00       	nop
 4b2:	00 00       	nop
 4b4:	44 00       	.word	0x0044	; ????
 4b6:	93 00       	.word	0x0093	; ????
 4b8:	14 00       	.word	0x0014	; ????
 4ba:	00 00       	nop
 4bc:	00 00       	nop
 4be:	00 00       	nop
 4c0:	44 00       	.word	0x0044	; ????
 4c2:	94 00       	.word	0x0094	; ????
 4c4:	14 00       	.word	0x0014	; ????
 4c6:	00 00       	nop
 4c8:	00 00       	nop
 4ca:	00 00       	nop
 4cc:	44 00       	.word	0x0044	; ????
 4ce:	95 00       	.word	0x0095	; ????
 4d0:	14 00       	.word	0x0014	; ????
 4d2:	00 00       	nop
 4d4:	00 00       	nop
 4d6:	00 00       	nop
 4d8:	44 00       	.word	0x0044	; ????
 4da:	96 00       	.word	0x0096	; ????
 4dc:	14 00       	.word	0x0014	; ????
 4de:	00 00       	nop
 4e0:	00 00       	nop
 4e2:	00 00       	nop
 4e4:	44 00       	.word	0x0044	; ????
 4e6:	97 00       	.word	0x0097	; ????
 4e8:	14 00       	.word	0x0014	; ????
 4ea:	00 00       	nop
 4ec:	00 00       	nop
 4ee:	00 00       	nop
 4f0:	44 00       	.word	0x0044	; ????
 4f2:	98 00       	.word	0x0098	; ????
 4f4:	14 00       	.word	0x0014	; ????
 4f6:	00 00       	nop
 4f8:	00 00       	nop
 4fa:	00 00       	nop
 4fc:	44 00       	.word	0x0044	; ????
 4fe:	99 00       	.word	0x0099	; ????
 500:	14 00       	.word	0x0014	; ????
 502:	00 00       	nop
 504:	00 00       	nop
 506:	00 00       	nop
 508:	44 00       	.word	0x0044	; ????
 50a:	9a 00       	.word	0x009a	; ????
 50c:	14 00       	.word	0x0014	; ????
 50e:	00 00       	nop
 510:	00 00       	nop
 512:	00 00       	nop
 514:	44 00       	.word	0x0044	; ????
 516:	9b 00       	.word	0x009b	; ????
 518:	14 00       	.word	0x0014	; ????
 51a:	00 00       	nop
 51c:	00 00       	nop
 51e:	00 00       	nop
 520:	44 00       	.word	0x0044	; ????
 522:	9c 00       	.word	0x009c	; ????
 524:	14 00       	.word	0x0014	; ????
 526:	00 00       	nop
 528:	00 00       	nop
 52a:	00 00       	nop
 52c:	44 00       	.word	0x0044	; ????
 52e:	9d 00       	.word	0x009d	; ????
 530:	14 00       	.word	0x0014	; ????
 532:	00 00       	nop
 534:	00 00       	nop
 536:	00 00       	nop
 538:	44 00       	.word	0x0044	; ????
 53a:	9e 00       	.word	0x009e	; ????
 53c:	14 00       	.word	0x0014	; ????
 53e:	00 00       	nop
 540:	00 00       	nop
 542:	00 00       	nop
 544:	44 00       	.word	0x0044	; ????
 546:	9f 00       	.word	0x009f	; ????
 548:	14 00       	.word	0x0014	; ????
 54a:	00 00       	nop
 54c:	00 00       	nop
 54e:	00 00       	nop
 550:	44 00       	.word	0x0044	; ????
 552:	a0 00       	.word	0x00a0	; ????
 554:	14 00       	.word	0x0014	; ????
 556:	00 00       	nop
 558:	00 00       	nop
 55a:	00 00       	nop
 55c:	44 00       	.word	0x0044	; ????
 55e:	a1 00       	.word	0x00a1	; ????
 560:	14 00       	.word	0x0014	; ????
 562:	00 00       	nop
 564:	00 00       	nop
 566:	00 00       	nop
 568:	44 00       	.word	0x0044	; ????
 56a:	a2 00       	.word	0x00a2	; ????
 56c:	14 00       	.word	0x0014	; ????
 56e:	00 00       	nop
 570:	00 00       	nop
 572:	00 00       	nop
 574:	44 00       	.word	0x0044	; ????
 576:	a3 00       	.word	0x00a3	; ????
 578:	14 00       	.word	0x0014	; ????
 57a:	00 00       	nop
 57c:	00 00       	nop
 57e:	00 00       	nop
 580:	44 00       	.word	0x0044	; ????
 582:	a4 00       	.word	0x00a4	; ????
 584:	14 00       	.word	0x0014	; ????
 586:	00 00       	nop
 588:	00 00       	nop
 58a:	00 00       	nop
 58c:	44 00       	.word	0x0044	; ????
 58e:	a5 00       	.word	0x00a5	; ????
 590:	14 00       	.word	0x0014	; ????
 592:	00 00       	nop
 594:	00 00       	nop
 596:	00 00       	nop
 598:	44 00       	.word	0x0044	; ????
 59a:	a6 00       	.word	0x00a6	; ????
 59c:	14 00       	.word	0x0014	; ????
 59e:	00 00       	nop
 5a0:	00 00       	nop
 5a2:	00 00       	nop
 5a4:	44 00       	.word	0x0044	; ????
 5a6:	a7 00       	.word	0x00a7	; ????
 5a8:	14 00       	.word	0x0014	; ????
 5aa:	00 00       	nop
 5ac:	00 00       	nop
 5ae:	00 00       	nop
 5b0:	44 00       	.word	0x0044	; ????
 5b2:	a8 00       	.word	0x00a8	; ????
 5b4:	14 00       	.word	0x0014	; ????
 5b6:	00 00       	nop
 5b8:	00 00       	nop
 5ba:	00 00       	nop
 5bc:	44 00       	.word	0x0044	; ????
 5be:	a9 00       	.word	0x00a9	; ????
 5c0:	14 00       	.word	0x0014	; ????
 5c2:	00 00       	nop
 5c4:	00 00       	nop
 5c6:	00 00       	nop
 5c8:	44 00       	.word	0x0044	; ????
 5ca:	aa 00       	.word	0x00aa	; ????
 5cc:	14 00       	.word	0x0014	; ????
 5ce:	00 00       	nop
 5d0:	00 00       	nop
 5d2:	00 00       	nop
 5d4:	44 00       	.word	0x0044	; ????
 5d6:	ab 00       	.word	0x00ab	; ????
 5d8:	14 00       	.word	0x0014	; ????
 5da:	00 00       	nop
 5dc:	00 00       	nop
 5de:	00 00       	nop
 5e0:	44 00       	.word	0x0044	; ????
 5e2:	ac 00       	.word	0x00ac	; ????
 5e4:	14 00       	.word	0x0014	; ????
 5e6:	00 00       	nop
 5e8:	00 00       	nop
 5ea:	00 00       	nop
 5ec:	44 00       	.word	0x0044	; ????
 5ee:	ad 00       	.word	0x00ad	; ????
 5f0:	14 00       	.word	0x0014	; ????
 5f2:	00 00       	nop
 5f4:	00 00       	nop
 5f6:	00 00       	nop
 5f8:	44 00       	.word	0x0044	; ????
 5fa:	ae 00       	.word	0x00ae	; ????
 5fc:	14 00       	.word	0x0014	; ????
 5fe:	00 00       	nop
 600:	00 00       	nop
 602:	00 00       	nop
 604:	44 00       	.word	0x0044	; ????
 606:	af 00       	.word	0x00af	; ????
 608:	14 00       	.word	0x0014	; ????
 60a:	00 00       	nop
 60c:	00 00       	nop
 60e:	00 00       	nop
 610:	44 00       	.word	0x0044	; ????
 612:	b0 00       	.word	0x00b0	; ????
 614:	14 00       	.word	0x0014	; ????
 616:	00 00       	nop
 618:	00 00       	nop
 61a:	00 00       	nop
 61c:	44 00       	.word	0x0044	; ????
 61e:	b1 00       	.word	0x00b1	; ????
 620:	14 00       	.word	0x0014	; ????
 622:	00 00       	nop
 624:	00 00       	nop
 626:	00 00       	nop
 628:	44 00       	.word	0x0044	; ????
 62a:	b2 00       	.word	0x00b2	; ????
 62c:	14 00       	.word	0x0014	; ????
 62e:	00 00       	nop
 630:	00 00       	nop
 632:	00 00       	nop
 634:	44 00       	.word	0x0044	; ????
 636:	b3 00       	.word	0x00b3	; ????
 638:	14 00       	.word	0x0014	; ????
 63a:	00 00       	nop
 63c:	00 00       	nop
 63e:	00 00       	nop
 640:	24 00       	.word	0x0024	; ????
 642:	00 00       	nop
 644:	14 00       	.word	0x0014	; ????
 646:	00 00       	nop
 648:	41 00       	.word	0x0041	; ????
 64a:	00 00       	nop
 64c:	24 00       	.word	0x0024	; ????
 64e:	c0 00       	.word	0x00c0	; ????
 650:	30 00       	.word	0x0030	; ????
 652:	00 00       	nop
 654:	00 00       	nop
 656:	00 00       	nop
 658:	44 00       	.word	0x0044	; ????
 65a:	c3 00       	.word	0x00c3	; ????
	...
 664:	24 00       	.word	0x0024	; ????
 666:	00 00       	nop
 668:	02 00       	.word	0x0002	; ????
 66a:	00 00       	nop
 66c:	00 00       	nop
 66e:	00 00       	nop
 670:	44 00       	.word	0x0044	; ????
 672:	d8 00       	.word	0x00d8	; ????
 674:	14 00       	.word	0x0014	; ????
 676:	00 00       	nop
 678:	00 00       	nop
 67a:	00 00       	nop
 67c:	44 00       	.word	0x0044	; ????
 67e:	d9 00       	.word	0x00d9	; ????
 680:	16 00       	.word	0x0016	; ????
 682:	00 00       	nop
 684:	00 00       	nop
 686:	00 00       	nop
 688:	44 00       	.word	0x0044	; ????
 68a:	da 00       	.word	0x00da	; ????
 68c:	18 00       	.word	0x0018	; ????
 68e:	00 00       	nop
 690:	00 00       	nop
 692:	00 00       	nop
 694:	44 00       	.word	0x0044	; ????
 696:	df 00       	.word	0x00df	; ????
 698:	1a 00       	.word	0x001a	; ????
 69a:	00 00       	nop
 69c:	00 00       	nop
 69e:	00 00       	nop
 6a0:	44 00       	.word	0x0044	; ????
 6a2:	24 01       	movw	r4, r8
 6a4:	2c 00       	.word	0x002c	; ????
 6a6:	00 00       	nop
 6a8:	00 00       	nop
 6aa:	00 00       	nop
 6ac:	44 00       	.word	0x0044	; ????
 6ae:	25 01       	movw	r4, r10
 6b0:	2e 00       	.word	0x002e	; ????
	...

Disassembly of section .stabstr:

00000000 <.stabstr>:
   0:	00 2f       	mov	r16, r16
   2:	74 6d       	ori	r23, 0xD4	; 212
   4:	70 2f       	mov	r23, r16
   6:	63 63       	ori	r22, 0x33	; 51
   8:	48 4f       	sbci	r20, 0xF8	; 248
   a:	6d 38       	cpi	r22, 0x8D	; 141
   c:	6a 39       	cpi	r22, 0x9A	; 154
   e:	2e 73       	andi	r18, 0x3E	; 62
  10:	00 76       	andi	r16, 0x60	; 96
  12:	6f 69       	ori	r22, 0x9F	; 159
  14:	64 3a       	cpi	r22, 0xA4	; 164
  16:	74 31       	cpi	r23, 0x14	; 20
  18:	3d 31       	cpi	r19, 0x1D	; 29
  1a:	00 5f       	subi	r16, 0xF0	; 240
  1c:	5f 76       	andi	r21, 0x6F	; 111
  1e:	65 63       	ori	r22, 0x35	; 53
  20:	74 6f       	ori	r23, 0xF4	; 244
  22:	72 73       	andi	r23, 0x32	; 50
  24:	3a 46       	sbci	r19, 0x6A	; 106
  26:	31 00       	.word	0x0031	; ????
  28:	2e 2e       	mov	r2, r30
  2a:	2f 2e       	mov	r2, r31
  2c:	2e 2f       	mov	r18, r30
  2e:	2e 2e       	mov	r2, r30
  30:	2f 2e       	mov	r2, r31
  32:	2e 2f       	mov	r18, r30
  34:	63 72       	andi	r22, 0x23	; 35
  36:	74 31       	cpi	r23, 0x14	; 20
  38:	2f 67       	ori	r18, 0x7F	; 127
  3a:	63 72       	andi	r22, 0x23	; 35
  3c:	74 31       	cpi	r23, 0x14	; 20
  3e:	2e 53       	subi	r18, 0x3E	; 62
  40:	00 5f       	subi	r16, 0xF0	; 240
  42:	5f 62       	ori	r21, 0x2F	; 47
  44:	61 64       	ori	r22, 0x41	; 65
  46:	5f 69       	ori	r21, 0x9F	; 159
  48:	6e 74       	andi	r22, 0x4E	; 78
  4a:	65 72       	andi	r22, 0x25	; 37
  4c:	72 75       	andi	r23, 0x52	; 82
  4e:	70 74       	andi	r23, 0x40	; 64
  50:	3a 46       	sbci	r19, 0x6A	; 106
  52:	31 00       	.word	0x0031	; ????

Disassembly of section .comment:

00000000 <.comment>:
   0:	47 43       	sbci	r20, 0x37	; 55
   2:	43 3a       	cpi	r20, 0xA3	; 163
   4:	20 28       	or	r2, r0
   6:	47 4e       	sbci	r20, 0xE7	; 231
   8:	55 29       	or	r21, r5
   a:	20 34       	cpi	r18, 0x40	; 64
   c:	2e 38       	cpi	r18, 0x8E	; 142
   e:	2e 32       	cpi	r18, 0x2E	; 46
	...
