; Shared ZMODEM wire primitives. UASM/TASM-compatible 80286 code.
; Header layout is five bytes: type followed by four position/flag bytes.
.286
ZM_JC_NEAR MACRO target
    LOCAL no_carry
    jnc     short no_carry
    jmp     target
no_carry:
ENDM
ZM_JNE_NEAR MACRO target
    LOCAL equal
    je      short equal
    jmp     target
equal:
ENDM

ZPAD    EQU 02Ah
ZDLE    EQU 018h
ZBIN    EQU 041h
ZHEX    EQU 042h
ZBIN32  EQU 043h
ZRUB0   EQU 06Ch
ZRUB1   EQU 06Dh
ZCRCE   EQU 068h
ZCRCG   EQU 069h
ZCRCQ   EQU 06Ah
ZCRCW   EQU 06Bh
ZRQINIT EQU 000h
ZRINIT  EQU 001h
ZSINIT  EQU 002h
ZACK    EQU 003h
ZFILE   EQU 004h
ZSKIP   EQU 005h
ZNAK    EQU 006h
ZABORT  EQU 007h
ZFIN    EQU 008h
ZRPOS   EQU 009h
ZDATA   EQU 00Ah
ZEOF    EQU 00Bh
ZFERR   EQU 00Ch
ZCRC    EQU 00Dh
ZCHALL  EQU 00Eh
ZCOMPL  EQU 00Fh
ZCAN    EQU 018h
ZPAD2   EQU 02Ah

; ZM_SendEscByte: AL byte. Preserves BX, uses flags only otherwise.
ZM_SendEscByte PROC NEAR
    cmp     al,0FFh
    je      short ZMSE_escape
    cmp     al,ZDLE
    je      short ZMSE_escape
    cmp     al,010h
    je      short ZMSE_escape
    cmp     al,011h
    je      short ZMSE_escape
    cmp     al,013h
    je      short ZMSE_escape
    cmp     al,090h
    je      short ZMSE_escape
    cmp     al,091h
    je      short ZMSE_escape
    cmp     al,093h
    je      short ZMSE_escape
    cmp     al,09Dh
    je      short ZMSE_escape
    jmp     UART_PutChar
ZMSE_escape:
    push    ax
    mov     al,ZDLE
    call    UART_PutChar
    pop     ax
    jc      short ZMSE_done
    xor     al,040h
    call    UART_PutChar
ZMSE_done:
    ret
ZM_SendEscByte ENDP

; ZM_GetEscByte: returns decoded byte in AL; CF on UART timeout/cancel.
ZM_GetEscByte PROC NEAR
    call    UART_GetChar
    jc      short ZMGE_done
    cmp     al,ZDLE
    jne     short ZMGE_plain
    call    UART_GetChar
    jc      short ZMGE_done
    cmp     al,ZRUB0
    je      short ZMGE_rub0
    cmp     al,ZRUB1
    je      short ZMGE_rub1
    cmp     al,ZCRCE
    je      short ZMGE_end
    cmp     al,ZCRCG
    je      short ZMGE_end
    cmp     al,ZCRCQ
    je      short ZMGE_end
    cmp     al,ZCRCW
    je      short ZMGE_end
    xor     al,040h
    clc
    ret
ZMGE_rub0:
    mov     al,07Fh
    clc
    ret
ZMGE_rub1:
    mov     al,0FFh
    clc
    ret
ZMGE_end:
    stc
ZMGE_done:
    ret
ZMGE_plain:
    clc
    ret
ZM_GetEscByte ENDP

; CRC16/XMODEM update: AL input byte, DX current CRC. Returns DX.
ZM_CRC16Update PROC NEAR
IFDEF BUFFERED_RECEIVE
    push    bx
    mov     bl,dh
    xor     bl,al
    xor     bh,bh
    shl     bx,1
    mov     dh,dl
    xor     dl,dl
    xor     dx,word ptr [BW_crc_table+bx]
    pop     bx
ELSE
    xor     dh,al
    push    cx
    mov     cl,8
ZM16_bit:
    shl     dx,1
    jnc     short ZM16_next
    xor     dx,01021h
ZM16_next:
    dec     cl
    jnz     short ZM16_bit
    pop     cx
ENDIF
    ret
ZM_CRC16Update ENDP

; CRC32/IEEE is updated as paired 16-bit words for 80286 compatibility.

; Send hex header. DS:SI -> 5-byte header. CRC is CRC16/XMODEM.
ZM_SendHexHeader PROC NEAR
    push    ax
    push    bx
    push    cx
    push    dx
    push    si
    mov     al,ZPAD
    call    UART_PutChar
    jc      short ZM_SH_bad
    mov     al,ZPAD
    call    UART_PutChar
    jc      short ZM_SH_bad
    mov     al,ZDLE
    call    UART_PutChar
    jc      short ZM_SH_bad
    mov     al,ZHEX
    call    UART_PutChar
    jc      short ZM_SH_bad
    xor     dx,dx
    mov     cx,5
ZM_SH_loop:
    lodsb
    push    ax
    call    ZM_CRC16Update
    pop     ax
    call    ZM_SendHexByte
    jc      short ZM_SH_bad
    loop    ZM_SH_loop
    xor     ax,ax
    call    ZM_CRC16Update
    call    ZM_CRC16Update
    mov     al,dh
    call    ZM_SendHexByte
    jc      short ZM_SH_bad
    mov     al,dl
    call    ZM_SendHexByte
    jc      short ZM_SH_bad
    mov     al,0Dh
    call    UART_PutChar
    jc      short ZM_SH_bad
    mov     al,0Ah
    call    UART_PutChar
    jc      short ZM_SH_bad
    mov     al,011h
    call    UART_PutChar
    jc      short ZM_SH_bad
    pop     si
    pop     dx
    pop     cx
    pop     bx
    pop     ax
    clc
    ret
ZM_SH_bad:
    pop     si
    pop     dx
    pop     cx
    pop     bx
    pop     ax
    stc
    ret
ZM_SendHexHeader ENDP

; Send standard binary16 header. DS:SI -> five header bytes.
ZM_SendBinHeader PROC NEAR
    push    ax
    push    bx
    push    cx
    push    dx
    push    si
    mov     al,ZPAD
    call    UART_PutChar
    jc      short ZM_SB_bad
    mov     al,ZDLE
    call    UART_PutChar
    jc      short ZM_SB_bad
    mov     al,ZBIN
    call    UART_PutChar
    jc      short ZM_SB_bad
    xor     dx,dx
    mov     cx,5
ZM_SB_header:
    lodsb
    push    ax
    call    ZM_CRC16Update
    pop     ax
    call    ZM_SendEscByte
    jc      short ZM_SB_bad
    loop    ZM_SB_header
    xor     ax,ax
    call    ZM_CRC16Update
    call    ZM_CRC16Update
    mov     al,dh
    call    ZM_SendEscByte
    jc      short ZM_SB_bad
    mov     al,dl
    call    ZM_SendEscByte
    jc      short ZM_SB_bad
    pop     si
    pop     dx
    pop     cx
    pop     bx
    pop     ax
    clc
    ret
ZM_SB_bad:
    pop     si
    pop     dx
    pop     cx
    pop     bx
    pop     ax
    stc
    ret
ZM_SendBinHeader ENDP

; AL byte -> two uppercase hexadecimal ASCII chars. Preserves AX/BX.
ZM_SendHexByte PROC NEAR
    push    ax
    push    bx
    mov     bl,al
    shr     al,1
    shr     al,1
    shr     al,1
    shr     al,1
    call    ZM_SendNibble
    jc      short ZM_HB_bad
    mov     al,bl
    and     al,0Fh
    call    ZM_SendNibble
    pop     bx
    pop     ax
    ret
ZM_HB_bad:
    pop     bx
    pop     ax
    stc
    ret
ZM_SendHexByte ENDP
ZM_SendNibble PROC NEAR
    and     al,0Fh
    cmp     al,10
    jb      short ZM_SN_num
    add     al,'A'-10
    jmp     short ZM_SN_send
ZM_SN_num:
    add     al,'0'
ZM_SN_send:
    jmp     UART_PutChar
ZM_SendNibble ENDP

; Receive next ZMODEM header. Skips noise; accepts hex, binary16 and binary32.
; Returns CF on error, DS:SI header[5], format is normalized to 5 bytes.
; Binary32 is accepted and CRC checked; positions are truncated only by caller.
ZM_RecvHeader PROC NEAR
    push    ax
    push    bx
    push    cx
    push    dx
    push    di
ZM_RH_seek:
IFDEF EXPERIMENTAL_BAUD
    call    BT_HeaderCheck
    ZM_JC_NEAR ZM_RH_bad
ENDIF
    call    UART_GetChar
    ZM_JC_NEAR ZM_RH_bad
    cmp     al,ZPAD
    jne     ZM_RH_seek
ZM_RH_pad:
IFDEF EXPERIMENTAL_BAUD
    call    BT_HeaderCheck
    ZM_JC_NEAR ZM_RH_bad
ENDIF
    call    UART_GetChar
    ZM_JC_NEAR ZM_RH_bad
    cmp     al,ZPAD
    je      short ZM_RH_pad
    cmp     al,ZDLE
    jne     ZM_RH_seek
    call    UART_GetChar
    ZM_JC_NEAR ZM_RH_bad
    mov     bl,al
IFDEF EXPERIMENTAL_BAUD
    mov     [zm_rxformat],al
ENDIF
    cmp     bl,ZHEX
    je      short ZM_RH_hex
    cmp     bl,ZBIN
    je      ZM_RH_bin16
    cmp     bl,ZBIN32
    je      short ZM_RH_bin32
    jmp     ZM_RH_seek
ZM_RH_bin32:
    mov     word ptr [zm_crc32_lo],0FFFFh
    mov     word ptr [zm_crc32_hi],0FFFFh
    mov     di,OFFSET zm_header
    mov     cx,5
ZM_RH32_data:
    call    ZM_GetEscByte
    ZM_JC_NEAR ZM_RH_bad
    stosb
    call    ZM_CRC32Update
    loop    ZM_RH32_data
    not     word ptr [zm_crc32_lo]
    not     word ptr [zm_crc32_hi]
    mov     ax,[zm_crc32_lo]
    mov     word ptr [zm_crc32_bytes],ax
    mov     ax,[zm_crc32_hi]
    mov     word ptr [zm_crc32_bytes+2],ax
    mov     si,OFFSET zm_crc32_bytes
    mov     cx,4
ZM_RH32_crc:
    call    ZM_GetEscByte
    ZM_JC_NEAR ZM_RH_bad
    cmp     al,[si]
    ZM_JNE_NEAR ZM_RH_bad
    inc     si
    loop    ZM_RH32_crc
    jmp     ZM_RH_ok
ZM_RH_hex:
    mov     di,OFFSET zm_header
    xor     dx,dx
    mov     cx,7
ZM_RH_hexloop:
IFDEF EXPERIMENTAL_BAUD
    call    BT_HeaderCheck
    ZM_JC_NEAR ZM_RH_bad
ENDIF
    call    ZM_GetHexByte
    ZM_JC_NEAR ZM_RH_bad
    cmp     cx,2
    jbe     short ZM_RH_hexcrc
    stosb
    call    ZM_CRC16Update
    jmp     short ZM_RH_hexnext
ZM_RH_hexcrc:
    mov     si,2
    sub     si,cx
    mov     [zm_rxcrc+si],al
ZM_RH_hexnext:
    loop    ZM_RH_hexloop
    xor     ax,ax
    call    ZM_CRC16Update
    call    ZM_CRC16Update
    mov     ax,dx
    xchg    al,ah
    cmp     al,[zm_rxcrc]
    ZM_JNE_NEAR ZM_RH_bad
    mov     al,ah
    cmp     al,[zm_rxcrc+1]
    ZM_JNE_NEAR ZM_RH_bad
    call    UART_GetChar
    ZM_JC_NEAR ZM_RH_bad
    jmp     short ZM_RH_ok
ZM_RH_bin16:
    mov     cx,7
    jmp     short ZM_RH_binary
ZM_RH_binary:
    mov     di,OFFSET zm_header
    xor     dx,dx
    mov     bx,cx
ZM_RH_binloop:
    call    ZM_GetEscByte
    ZM_JC_NEAR ZM_RH_bad
    cmp     bx,2
    jbe     short ZM_RH_bincrc
    cmp     bl,9
    jne     short ZM_RH_store
    ; In a binary32 header, only first five fields retained.
ZM_RH_store:
    cmp     di,OFFSET zm_header+5
    jae     short ZM_RH_update
    stosb
ZM_RH_update:
    call    ZM_CRC16Update
    jmp     short ZM_RH_bnext
ZM_RH_bincrc:
    mov     si,2
    sub     si,bx
    mov     [zm_rxcrc+si],al
ZM_RH_bnext:
    dec     bx
    jnz     ZM_RH_binloop
    xor     ax,ax
    call    ZM_CRC16Update
    call    ZM_CRC16Update
    mov     ax,dx
    xchg    al,ah
    cmp     al,[zm_rxcrc]
    ZM_JNE_NEAR ZM_RH_bad
    cmp     ah,[zm_rxcrc+1]
    ZM_JNE_NEAR ZM_RH_bad
    clc
ZM_RH_ok:
    mov     si,OFFSET zm_header
    pop     di
    pop     dx
    pop     cx
    pop     bx
    pop     ax
    clc
    ret
ZM_RH_bad:
    pop     di
    pop     dx
    pop     cx
    pop     bx
    pop     ax
    stc
    ret
ZM_RecvHeader ENDP

ZM_CRC32Update PROC NEAR
    push    bx
    push    cx
    xor     byte ptr [zm_crc32_lo],al
    mov     cl,8
ZM_CRC32_bit:
    shr     word ptr [zm_crc32_hi],1
    rcr     word ptr [zm_crc32_lo],1
    jnc     short ZM_CRC32_next
    xor     word ptr [zm_crc32_hi],0EDB8h
    xor     word ptr [zm_crc32_lo],08320h
ZM_CRC32_next:
    dec     cl
    jnz     short ZM_CRC32_bit
    pop     cx
    pop     bx
    ret
ZM_CRC32Update ENDP

ZM_GetHexByte PROC NEAR
    push    bx
    call    UART_GetChar
    jc      short ZM_GH_bad
    call    ZM_HexNibble
    jc      short ZM_GH_bad
    mov     bl,al
    shl     bl,1
    shl     bl,1
    shl     bl,1
    shl     bl,1
    call    UART_GetChar
    jc      short ZM_GH_bad
    call    ZM_HexNibble
    jc      short ZM_GH_bad
    or      al,bl
    pop     bx
    clc
    ret
ZM_GH_bad:
    pop     bx
    stc
    ret
ZM_GetHexByte ENDP
ZM_HexNibble PROC NEAR
    cmp     al,'0'
    jb      short ZM_HN_bad
    cmp     al,'9'
    jbe     short ZM_HN_num
    and     al,0DFh
    cmp     al,'A'
    jb      short ZM_HN_bad
    cmp     al,'F'
    ja      short ZM_HN_bad
    sub     al,'A'-10
    clc
    ret
ZM_HN_num:
    sub     al,'0'
    clc
    ret
ZM_HN_bad:
    stc
    ret
ZM_HexNibble ENDP

; Receive CRC16 data subpacket into DS:DI up to CX bytes; returns count in CX,
; CF indicates framing/CRC failure. ZM terminator is returned in zm_endchar.
ZM_RecvData16 PROC NEAR
    push    ax
    push    bx
    push    dx
    xor     dx,dx
    xor     bx,bx
ZM_RD_loop:
    call    ZM_GetEscByte
    jc      short ZM_RD_terminator
    cmp     bx,cx
    jae     short ZM_RD_overflow
    stosb
    inc     bx
    call    ZM_CRC16Update
    jmp     short ZM_RD_loop
ZM_RD_terminator:
    mov     [zm_endchar],al
    call    ZM_CRC16Update
    xor     ax,ax
    call    ZM_CRC16Update
    call    ZM_CRC16Update
    mov     cx,2
ZM_RD_crc:
    call    ZM_GetEscByte
    jc      short ZM_RD_bad
    cmp     cx,2
    jne     short ZM_RD_crc2
    mov     [zm_rxcrc],al
    jmp     short ZM_RD_crcnext
ZM_RD_crc2:
    mov     [zm_rxcrc+1],al
ZM_RD_crcnext:
    loop    ZM_RD_crc
    mov     ax,dx
    xchg    al,ah
    cmp     al,[zm_rxcrc]
    jne     short ZM_RD_bad
    cmp     ah,[zm_rxcrc+1]
    jne     short ZM_RD_bad
    mov     cx,bx
    pop     dx
    pop     bx
    pop     ax
    clc
    ret
ZM_RD_overflow:
    stc
    jmp     short ZM_RD_exit
ZM_RD_bad:
    stc
ZM_RD_exit:
    pop     dx
    pop     bx
    pop     ax
    ret
ZM_RecvData16 ENDP

zm_header DB 5 DUP(0)
IFDEF EXPERIMENTAL_BAUD
zm_rxformat DB 0
ENDIF
zm_rxcrc DB 2 DUP(0)
zm_endchar DB 0

; Send DS:SI, CX payload bytes as one CRC16 subpacket. AL is ZCRCE/G/W.
ZM_SendData16 PROC NEAR
    push    ax
    push    bx
    push    cx
    push    dx
    push    si
    mov     bl,al
    xor     dx,dx
ZM_SD_loop:
    jcxz    short ZM_SD_tail
    lodsb
    push    ax
    call    ZM_CRC16Update
    pop     ax
    call    ZM_SendEscByte
    jc      short ZM_SD_bad
    loop    ZM_SD_loop
ZM_SD_tail:
    mov     al,bl
    call    ZM_CRC16Update
    xor     ax,ax
    call    ZM_CRC16Update
    call    ZM_CRC16Update
    mov     al,ZDLE
    call    UART_PutChar
    jc      short ZM_SD_bad
    mov     al,bl
    call    UART_PutChar
    jc      short ZM_SD_bad
    mov     al,dh
    call    ZM_SendEscByte
    jc      short ZM_SD_bad
    mov     al,dl
    call    ZM_SendEscByte
    jc      short ZM_SD_bad
    pop     si
    pop     dx
    pop     cx
    pop     bx
    pop     ax
    clc
    ret
ZM_SD_bad:
    pop     si
    pop     dx
    pop     cx
    pop     bx
    pop     ax
    stc
    ret
ZM_SendData16 ENDP

; Build a header from zm_type and 32-bit byte offset zm_pos_hi:zm_pos_lo.
ZM_MakeHeader PROC NEAR
    mov     al,[zm_type]
    mov     [zm_header],al
    mov     ax,[zm_pos_lo]
    mov     [zm_header+1],al
    mov     [zm_header+2],ah
    mov     ax,[zm_pos_hi]
    mov     [zm_header+3],al
    mov     [zm_header+4],ah
    ret
ZM_MakeHeader ENDP

zm_type   DB 0
zm_pos_lo DW 0
zm_pos_hi DW 0
zm_crc32_lo DW 0
zm_crc32_hi DW 0
zm_crc32_bytes DD 0
