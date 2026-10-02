; ZRECVY only: standard ZDATA with CRC16 G...G,W subpackets. The advertised
; capacity counts decoded wire payload, not escaping/framing. No DOS work or
; decompression occurs until the entire <=60KiB window is CRC-validated.
; A bad window is discarded before any stream/decompressor state is changed.

BW_Allocate PROC NEAR
    push    ds
    pop     es
    mov     bx,OFFSET BW_image_end
    add     bx,15
    mov     cl,4
    shr     bx,cl
    mov     ah,4Ah
    int     21h
    jc      short BWA_done
    mov     bx,1000h                 ; full 64KiB allocation, 60KiB advertised
    mov     ah,48h
    int     21h
    jc      short BWA_done
    mov     [BW_segment],ax
    xor     bx,bx
    mov     si,OFFSET BW_crc_table
BWA_entry:
    mov     dh,bl
    xor     dl,dl
    mov     cx,8
BWA_bit:
    shl     dx,1
    jnc     short BWA_next
    xor     dx,1021h
BWA_next:
    loop    BWA_bit
    mov     [si],dx
    add     si,2
    inc     bx
    cmp     bx,256
    jb      short BWA_entry
    clc
BWA_done:
    ret
BW_Allocate ENDP

; ES is restored for the metadata parser and other single-segment services.
BW_Receive PROC NEAR
    push    es
    mov     ax,[BW_segment]
    mov     es,ax
    xor     di,di
    mov     word ptr [BW_count],0
BWR_packet:
    xor     dx,dx                    ; per-subpacket CRC
    xor     bx,bx                    ; per-subpacket payload length
BWR_byte:
    call    BW_GetEsc
    F_JC BWR_bad
    cmp     byte ptr [BW_marker],0
    jne     short BWR_end
    cmp     di,RECV_WINDOW
    F_JAE BWR_bad
    cmp     bx,512
    F_JAE BWR_bad
    stosb                           ; directly into receive segment
    inc     bx
    call    ZM_CRC16Update
    jmp     short BWR_byte
BWR_end:
    mov     [zm_endchar],al
    call    ZM_CRC16Update
    xor     ax,ax
    call    ZM_CRC16Update
    call    ZM_CRC16Update
    call    BW_GetEsc
    F_JC BWR_bad
    cmp     byte ptr [BW_marker],0
    F_JNE BWR_bad
    cmp     al,dh
    F_JNE BWR_bad
    call    BW_GetEsc
    F_JC BWR_bad
    cmp     byte ptr [BW_marker],0
    F_JNE BWR_bad
    cmp     al,dl
    F_JNE BWR_bad
    cmp     byte ptr [zm_endchar],ZCRCW
    je      short BWR_done
    cmp     byte ptr [zm_endchar],ZCRCG
    F_JNE BWR_bad
    or      bx,bx                    ; disallow unbounded empty continuation
    F_JZ BWR_bad
    cmp     di,RECV_WINDOW
    F_JAE BWR_bad
    jmp     BWR_packet
BWR_done:
    mov     [BW_count],di
    pop     es
    clc
    ret
BWR_bad:
    pop     es
    stc
    ret
BW_Receive ENDP

; Unlike ZM_GetEscByte, carry means ONLY timeout/cancel, never a terminator.
BW_GetEsc PROC NEAR
    mov     byte ptr [BW_marker],0
    call    UART_GetChar
    jc      short BWG_done
    cmp     al,ZDLE
    jne     short BWG_plain
    call    UART_GetChar
    jc      short BWG_done
    cmp     al,ZCRCE
    jb      short BWG_escape
    cmp     al,ZCRCW
    ja      short BWG_escape
    mov     byte ptr [BW_marker],1
    clc
    ret
BWG_escape:
    cmp     al,ZRUB0
    je      short BWG_rub0
    cmp     al,ZRUB1
    je      short BWG_rub1
    xor     al,40h
    clc
    ret
BWG_rub0:
    mov     al,7Fh
    clc
    ret
BWG_rub1:
    mov     al,0FFh
BWG_plain:
    clc
BWG_done:
    ret
BW_GetEsc ENDP

; Sender has stopped at ZCRCW. Compressed block carry persists across windows;
; append_compressed reads ES directly, retaining its existing bounded block
; assembler/decompressor. Raw data goes straight from the segment to DOS.
BW_Commit PROC NEAR
    push    es
    mov     ax,[BW_segment]
    mov     es,ax
    mov     cx,[BW_count]
    mov     [data_count],cx
    cmp     byte ptr [compressed_flag],0
    je      short BWC_raw
    call    append_compressed
    jc      short BWC_bad
    jmp     short BWC_advance
BWC_raw:
    mov     ax,[decoded_lo]
    mov     dx,[decoded_hi]
    add     ax,cx
    adc     dx,0
    jc      short BWC_bad
    cmp     dx,[expected_hi]
    ja      short BWC_bad
    jb      short BWC_write
    cmp     ax,[expected_lo]
    ja      short BWC_bad
BWC_write:
    mov     bx,[out_handle]
    xor     dx,dx
    push    ds
    push    es
    pop     ds
    mov     ah,40h
    int     21h
    pop     ds
    jc      short BWC_bad
    cmp     ax,[BW_count]
    jne     short BWC_bad
    add     [decoded_lo],ax
    adc     word ptr [decoded_hi],0
BWC_advance:
    mov     ax,[BW_count]
    add     [out_pos_lo],ax
    adc     word ptr [out_pos_hi],0
    jc      short BWC_bad
    pop     es
    clc
    ret
BWC_bad:
    pop     es
    stc
    ret
BW_Commit ENDP

; A sender cannot transmit more than one window without permission. Drain the
; damaged window through one UART idle timeout before replying ZRPOS; do not
; mistake a corrupt/missing terminator for a safe disk-service boundary.
; Bound even hostile continuous input to 128KiB (worst-case escaping + CRCs).
BW_Drain PROC NEAR
    mov     bx,2
    xor     cx,cx
BWD_next:
    call    UART_GetChar
    jc      short BWD_idle
    loop    BWD_next
    dec     bx
    jnz     short BWD_next
    stc
    ret
BWD_idle:
    cmp     byte ptr cs:[UART_CANCEL],0
    jne     short BWD_cancel
    call    UART_PollCancel
    ret
BWD_cancel:
    stc
    ret
BW_Drain ENDP

BW_segment DW 0
BW_count DW 0
BW_retries DB 0
BW_marker DB 0
BW_memory_msg DB 'ZRECVY: unable to allocate 64KiB receive memory.',13,10,'$'
BW_crc_table DW 256 DUP(0)
