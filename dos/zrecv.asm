; DOS real-mode ZMODEM receiver. UASM, 80286 instruction set only.
.286
F_JC MACRO target
    LOCAL skip
    jnc     short skip
    jmp     target
skip:
ENDM
F_JNE MACRO target
    LOCAL skip
    je      short skip
    jmp     target
skip:
ENDM
F_JE MACRO target
    LOCAL skip
    jne     short skip
    jmp     target
skip:
ENDM
F_JZ MACRO target
    LOCAL skip
    jnz     short skip
    jmp     target
skip:
ENDM
F_JNZ MACRO target
    LOCAL skip
    jz      short skip
    jmp     target
skip:
ENDM
F_JA MACRO target
    LOCAL skip
    jbe     short skip
    jmp     target
skip:
ENDM
F_JAE MACRO target
    LOCAL skip
    jb      short skip
    jmp     target
skip:
ENDM
F_JB MACRO target
    LOCAL skip
    jae     short skip
    jmp     target
skip:
ENDM
.model tiny
.code
ORG 100h
start:
    jmp     main

include serial.inc
include zmodem.inc
include zx0.inc
include filemeta.inc
IFDEF EXPERIMENTAL_BAUD
include training.inc
RECV_FLAGS EQU 0C0h
ELSE
RECV_FLAGS EQU 080h
ENDIF
IFDEF BUFFERED_RECEIVE
RECV_WINDOW EQU 61440
include buffered.inc
ELSE
RECV_WINDOW EQU 512
ENDIF

main PROC NEAR
    cld
    push    cs
    pop     ds
IFDEF BUFFERED_RECEIVE
    ; Move the stack into the retained image BEFORE shrinking the PSP block.
    cli
    mov     ax,cs
    mov     ss,ax
    mov     sp,OFFSET BW_stack_top
    sti
    call    BW_Allocate
    jnc     short recv_memory_ok
    mov     dx,OFFSET BW_memory_msg
    mov     ah,09h
    int     21h
    mov     ax,4C01h
    int     21h
recv_memory_ok:
ENDIF
    call    UART_Init
    F_JC recv_fail
    xor     dx,dx
    mov     ax,UART_BAUD_9600
    call    UART_SetBaud
    F_JC recv_fail
    mov     dx,OFFSET ready_msg
    mov     ah,09h
    int     21h
    mov     byte ptr [zm_type],ZRINIT
    mov     word ptr [zm_pos_lo],RECV_WINDOW
    mov     word ptr [zm_pos_hi],0
    call    ZM_MakeHeader
    mov     byte ptr [zm_header+4],RECV_FLAGS
    mov     si,OFFSET zm_header
    call    ZM_SendHexHeader
    F_JC recv_fail
recv_header:
    call    ZM_RecvHeader
    F_JC recv_wait_retry
IFDEF EXPERIMENTAL_BAUD
    cmp     byte ptr [zm_rxformat],ZHEX
    jne     short recv_not_training
    cmp     byte ptr [zm_header],18
    jne     short recv_not_training
    cmp     word ptr [zm_header+1],0425Ah
    jne     short recv_not_training
    cmp     word ptr [zm_header+3],03154h
    jne     short recv_not_training
    cmp     byte ptr [BT_locked],0
    jne     short recv_not_training
    cmp     byte ptr [BT_can_train],0
    je      short recv_not_training
    call    BT_Train
    F_JC recv_fail
    jmp     send_ready
recv_not_training:
ENDIF
    cmp     byte ptr [zm_header],ZFILE
    F_JE receive_file
    cmp     byte ptr [zm_header],ZFIN
    F_JE receive_finish
    cmp     byte ptr [zm_header],ZRQINIT
    F_JE send_ready
    jmp     recv_header
recv_wait_retry:
    call    UART_PollCancel
    F_JC recv_fail
IFDEF EXPERIMENTAL_BAUD
    cmp     byte ptr [BT_locked],0
    je      short recv_not_locked
    cmp     byte ptr [BT_header_wait],0
    F_JE recv_fail
    call    BT_Baseline
    F_JC recv_fail
recv_not_locked:
ENDIF
    jmp     send_ready

send_ready:
    mov     byte ptr [zm_type],ZRINIT
    mov     word ptr [zm_pos_lo],RECV_WINDOW
    mov     word ptr [zm_pos_hi],0
    call    ZM_MakeHeader
    mov     byte ptr [zm_header+4],RECV_FLAGS
    mov     si,OFFSET zm_header
    call    ZM_SendHexHeader
    F_JC recv_fail
    jmp     recv_header

receive_file:
IFDEF EXPERIMENTAL_BAUD
    mov     byte ptr [BT_can_train],0
    mov     byte ptr [BT_header_wait],0
ENDIF
    mov     di,OFFSET metadata
    mov     cx,64
    call    ZM_RecvData16
    F_JC reject_file
    mov     [metadata_count],cx
    cmp     byte ptr [zm_endchar],ZCRCW
    F_JNE reject_file
    ; Parse bounded DOS 8.3 basename, decimal byte count and ZX0 request flag.
    call    parse_metadata
    F_JC reject_file
    mov     dx,OFFSET out_name
    mov     cx,0
    mov     ah,5Bh
    int     21h
    F_JC reject_file
    mov     [out_handle],ax
    mov     byte ptr [file_created],1
    mov     word ptr [out_pos_lo],0
    mov     word ptr [out_pos_hi],0
IFDEF BUFFERED_RECEIVE
    mov     byte ptr [BW_retries],0
ENDIF
    mov     byte ptr [zm_type],ZRPOS
    mov     word ptr [zm_pos_lo],0
    mov     word ptr [zm_pos_hi],0
    call    ZM_MakeHeader
    mov     si,OFFSET zm_header
    call    ZM_SendHexHeader
    F_JC file_error

receive_data_header:
    call    ZM_RecvHeader
IFDEF BUFFERED_RECEIVE
    F_JC resend_position
ELSE
    F_JC file_error
ENDIF
    cmp     byte ptr [zm_header],ZDATA
    jne     short check_eof_header
    mov     ax,word ptr [zm_header+1]
    cmp     ax,[out_pos_lo]
    F_JNE resend_position
    mov     ax,word ptr [zm_header+3]
    cmp     ax,[out_pos_hi]
    F_JNE resend_position
    jmp     receive_packet
check_eof_header:
    cmp     byte ptr [zm_header],ZEOF
    F_JE receive_eof
    cmp     byte ptr [zm_header],ZFILE
    F_JE file_error
    jmp     receive_data_header

receive_packet:
IFDEF BUFFERED_RECEIVE
    call    BW_Receive
    F_JC resend_position
    ; All CRCs have passed and the sender is stopped at ZCRCW.
    call    BW_Commit
    F_JC file_error
    mov     byte ptr [BW_retries],0
    jmp     send_ack
ELSE
    mov     di,OFFSET data_buf
    mov     cx,512
    call    ZM_RecvData16
    F_JC resend_position
    mov     [data_count],cx
    cmp     byte ptr [compressed_flag],0
    jne     short packet_compressed
    mov     ax,[decoded_lo]
    mov     dx,[decoded_hi]
    add     ax,cx
    adc     dx,0
    cmp     dx,[expected_hi]
    F_JA file_error
    jb      short uncompressed_size_ok
    cmp     ax,[expected_lo]
    F_JA file_error
uncompressed_size_ok:
    mov     bx,[out_handle]
    mov     dx,OFFSET data_buf
    mov     ah,40h
    int     21h
    F_JC file_error
    cmp     ax,[data_count]
    F_JNE file_error
    mov     ax,[data_count]
    add     [decoded_lo],ax
    jnc     short decoded_added
    inc     word ptr [decoded_hi]
decoded_added:
    jmp     short packet_wire_advance
packet_compressed:
    call    append_compressed
    F_JC file_error
packet_wire_advance:
    mov     ax,[data_count]
    add     [out_pos_lo],ax
    jnc     short packet_pos_ok
    inc     word ptr [out_pos_hi]
packet_pos_ok:
    cmp     byte ptr [zm_endchar],ZCRCE
    jne     short check_zcrcg
    jmp     receive_data_header
check_zcrcg:
    cmp     byte ptr [zm_endchar],ZCRCG
    jne     short check_zcrcq
    jmp     receive_packet
check_zcrcq:
    cmp     byte ptr [zm_endchar],ZCRCQ
    je      short send_ack
    cmp     byte ptr [zm_endchar],ZCRCW
    F_JNE file_error
ENDIF
send_ack:
    mov     byte ptr [zm_type],ZACK
    mov     ax,[out_pos_lo]
    mov     [zm_pos_lo],ax
    mov     ax,[out_pos_hi]
    mov     [zm_pos_hi],ax
    call    ZM_MakeHeader
    mov     si,OFFSET zm_header
    call    ZM_SendHexHeader
    F_JC file_error
    cmp     byte ptr [zm_endchar],ZCRCW
    jne     short continue_zdata
    jmp     receive_data_header
continue_zdata:
    jmp     receive_packet

resend_position:
IFDEF BUFFERED_RECEIVE
    inc     byte ptr [BW_retries]
    cmp     byte ptr [BW_retries],5
    F_JA file_error
    call    BW_Drain
    F_JC file_error
ENDIF
    mov     byte ptr [zm_type],ZRPOS
    mov     ax,[out_pos_lo]
    mov     [zm_pos_lo],ax
    mov     ax,[out_pos_hi]
    mov     [zm_pos_hi],ax
    call    ZM_MakeHeader
    mov     si,OFFSET zm_header
    call    ZM_SendHexHeader
    F_JC file_error
    jmp     receive_data_header

receive_eof:
    mov     ax,word ptr [zm_header+1]
    cmp     ax,[out_pos_lo]
    F_JNE file_error
    mov     ax,word ptr [zm_header+3]
    cmp     ax,[out_pos_hi]
    F_JNE file_error
    cmp     word ptr [block_count],0
    F_JNE file_error
    mov     ax,[decoded_lo]
    cmp     ax,[expected_lo]
    F_JNE file_error
    mov     ax,[decoded_hi]
    cmp     ax,[expected_hi]
    F_JNE file_error
    mov     bx,[out_handle]
    mov     cx,[file_time]
    mov     dx,[file_date]
    mov     ax,5701h
    int     21h
    F_JC file_error
    mov     ah,3Eh
    int     21h
    F_JC file_error
    mov     word ptr [out_handle],0FFFFh
    xor     cx,cx
    mov     cl,[file_attr]
    mov     dx,OFFSET out_name
    mov     ax,4301h
    int     21h
    F_JC file_error
    mov     byte ptr [file_created],0
IFNDEF EXPERIMENTAL_BAUD
    ; Finish DOS/BIOS output before granting the sender permission to send
    ; ZFIN (or another file). Otherwise its header overruns a small RX FIFO
    ; while the serial console is busy printing this message.
    mov     dx,OFFSET file_done
    mov     ah,09h
    int     21h
ELSE
    mov     byte ptr [BT_file_done],1
ENDIF
    mov     byte ptr [zm_type],ZRINIT
    mov     word ptr [zm_pos_lo],RECV_WINDOW
    mov     word ptr [zm_pos_hi],0
    call    ZM_MakeHeader
    mov     byte ptr [zm_header+4],RECV_FLAGS
    mov     si,OFFSET zm_header
    call    ZM_SendHexHeader
    F_JC recv_fail
    jmp     recv_header

reject_file:
    mov     byte ptr [zm_type],ZSKIP
    mov     word ptr [zm_pos_lo],0
    mov     word ptr [zm_pos_hi],0
    call    ZM_MakeHeader
    mov     si,OFFSET zm_header
    call    ZM_SendHexHeader
    jmp     recv_header
file_error:
    mov     bx,[out_handle]
    cmp     bx,0FFFFh
    je      short file_delete
    mov     ah,3Eh
    int     21h
    mov     word ptr [out_handle],0FFFFh
file_delete:
    cmp     byte ptr [file_created],0
    F_JE recv_fail
    ; Only this session's exclusive create is eligible for deletion.
    ; Clear readonly too, in case setting attributes failed after applying it.
    mov     dx,OFFSET out_name
    xor     cx,cx
    mov     ax,4301h
    int     21h
    mov     dx,OFFSET out_name
    mov     ah,41h
    int     21h
recv_fail:
IFDEF EXPERIMENTAL_BAUD
    call    BT_RestoreConsole
ENDIF
    call    UART_Restore
    mov     dx,OFFSET fail_msg
    mov     ah,09h
    int     21h
    mov     ax,4C01h
    int     21h

receive_finish:
    mov     byte ptr [zm_type],ZFIN
    mov     word ptr [zm_pos_lo],0
    mov     word ptr [zm_pos_hi],0
    call    ZM_MakeHeader
    mov     si,OFFSET zm_header
    call    ZM_SendHexHeader
    F_JC recv_fail
    call    UART_GetChar
    F_JC recv_fail
    cmp     al,0Ah
    F_JNE recv_fail
    call    UART_GetChar
    F_JC recv_fail
    cmp     al,011h
    jne     short finish_first_o
    call    UART_GetChar
    F_JC recv_fail
finish_first_o:
    cmp     al,'O'
    F_JNE recv_fail
    call    UART_GetChar
    F_JC recv_fail
    cmp     al,'O'
    F_JNE recv_fail
IFDEF EXPERIMENTAL_BAUD
    mov     cx,4
    call    BT_WaitTicks
    F_JC recv_fail
    call    BT_RestoreConsole
ENDIF
    call    UART_Restore
IFDEF EXPERIMENTAL_BAUD
    cmp     byte ptr [BT_file_done],0
    je      short recv_exit_ok
    mov     dx,OFFSET file_done
    mov     ah,09h
    int     21h
ENDIF
recv_exit_ok:
    mov     ax,4C00h
    int     21h
main ENDP
parse_metadata PROC NEAR
    push    ax
    push    bx
    push    cx
    push    dx
    push    si
    push    di
    push    bp
    mov     si,OFFSET metadata
    mov     di,OFFSET out_name
    xor     bx,bx                    ; base-name length
    xor     bp,bp                    ; extension length
    mov     byte ptr [name_dot],0
PM_name:
    mov     ax,si
    sub     ax,OFFSET metadata
    cmp     ax,[metadata_count]
    F_JAE PM_bad
    lodsb
    or      al,al
    jz      short PM_name_end
    cmp     al,'.'
    jne     short PM_char
    cmp     byte ptr [name_dot],0
    F_JNE PM_bad
    or      bx,bx
    F_JZ PM_bad
    mov     byte ptr [name_dot],1
    mov     al,'.'
    stosb
    jmp     PM_name
PM_char:
    cmp     al,'A'
    jb      short PM_lower
    cmp     al,'Z'
    jbe     short PM_store
PM_lower:
    cmp     al,'a'
    jb      short PM_digit
    cmp     al,'z'
    ja      short PM_digit
    and     al,0DFh
    jmp     short PM_store
PM_digit:
    cmp     al,'0'
    jb      short PM_punct
    cmp     al,'9'
    jbe     short PM_store
PM_punct:
    cmp     al,'_'
    je      short PM_store
    cmp     al,'-'
    F_JNE PM_bad
PM_store:
    cmp     byte ptr [name_dot],0
    jne     short PM_ext
    inc     bx
    cmp     bx,8
    F_JA PM_bad
    jmp     short PM_copy
PM_ext:
    inc     bp
    cmp     bp,3
    F_JA PM_bad
PM_copy:
    stosb
    jmp     PM_name
PM_name_end:
    or      bx,bx
    F_JZ PM_bad
    cmp     byte ptr [name_dot],0
    je      short PM_name_term
    or      bp,bp
    F_JZ PM_bad
PM_name_term:
    xor     al,al
    stosb
    mov     word ptr [expected_lo],0
    mov     word ptr [expected_hi],0
    xor     cx,cx                    ; at least one size digit
PM_size:
    mov     ax,si
    sub     ax,OFFSET metadata
    cmp     ax,[metadata_count]
    F_JAE PM_bad
    lodsb
    or      al,al
    jz      short PM_size_end
    cmp     al,'0'
    F_JB PM_bad
    cmp     al,'9'
    F_JA PM_bad
    sub     al,'0'
    mov     [size_digit],al
    mov     ax,[expected_lo]
    mov     bx,10
    mul     bx
    mov     [size_mul_hi],dx
    mov     [expected_lo],ax
    mov     ax,[expected_hi]
    mul     bx
    or      dx,dx
    F_JNZ PM_bad
    add     ax,[size_mul_hi]
    F_JC PM_bad
    mov     [expected_hi],ax
    xor     ah,ah
    mov     al,[size_digit]
    add     [expected_lo],ax
    jnc     short PM_digit_done
    inc     word ptr [expected_hi]
    F_JZ PM_bad
PM_digit_done:
    inc     cx
    jmp     short PM_size
PM_size_end:
    or      cx,cx
    F_JZ PM_bad
    ; Version, readonly, compression, reserved, time LE16, date LE16.
    mov     ax,si
    sub     ax,OFFSET metadata
    add     ax,8
    cmp     ax,[metadata_count]
    F_JA PM_bad
    cmp     byte ptr [si],1
    F_JNE PM_bad
    cmp     byte ptr [si+3],0
    F_JNE PM_bad
    mov     cx,[si+4]
    mov     dx,[si+6]
    call    MD_Validate
    F_JC PM_bad
    mov     [file_time],cx
    mov     [file_date],dx
    mov     al,[si+1]
    and     al,1
    mov     [file_attr],al
    mov     word ptr [decoded_lo],0
    mov     word ptr [decoded_hi],0
    mov     word ptr [block_count],0
    mov     word ptr [block_target],0
    mov     byte ptr [compressed_flag],0
    test    byte ptr [si+2],080h
    jz      short PM_done
    mov     byte ptr [compressed_flag],1
PM_done:
    pop     bp
    pop     di
    pop     si
    pop     dx
    pop     cx
    pop     bx
    pop     ax
    clc
    ret
PM_bad:
    pop     bp
    pop     di
    pop     si
    pop     dx
    pop     cx
    pop     bx
    pop     ax
    stc
    ret
parse_metadata ENDP

append_compressed PROC NEAR
    push    ax
    push    bx
    push    cx
    push    dx
    push    si
    push    di
IFDEF BUFFERED_RECEIVE
    xor     si,si
ELSE
    mov     si,OFFSET data_buf
ENDIF
    mov     cx,[data_count]
AC_next:
    or      cx,cx
    jnz     short AC_more
    jmp     AC_ok
AC_more:
    mov     bx,[block_count]
    cmp     bx,8196
    F_JAE AC_bad
IFDEF BUFFERED_RECEIVE
    mov     al,es:[si]
ELSE
    mov     al,[si]
ENDIF
    mov     [block_buf+bx],al
    inc     si
    inc     bx
    mov     [block_count],bx
    dec     cx
    mov     [packet_cursor],si
    mov     [packet_left],cx
    cmp     bx,4
    jb      AC_next
    cmp     word ptr [block_target],0
    jne     short AC_have_target
    mov     ax,word ptr [block_buf]
    or      ax,ax
    F_JZ AC_bad
    cmp     ax,ZX0_MAX_BLOCK
    F_JA AC_bad
    mov     dx,word ptr [block_buf+2]
    and     dx,07FFFh
    cmp     dx,8192
    F_JA AC_bad
    add     dx,4
    mov     [block_target],dx
AC_have_target:
    cmp     bx,[block_target]
    jb      AC_next
    mov     ax,word ptr [block_buf]
    mov     [block_decoded],ax
    mov     dx,word ptr [block_buf+2]
    test    dx,08000h
    jz      short AC_decode
    and     dx,07FFFh
    cmp     dx,ax
    F_JNE AC_bad
    mov     si,OFFSET block_buf+4
    mov     cx,dx
    mov     di,si
    jmp     short AC_write
AC_decode:
    and     dx,07FFFh
    mov     si,OFFSET block_buf+4
    mov     cx,dx
    mov     di,OFFSET decode_buf
    mov     bp,[block_decoded]
    push    es
    push    ds
    pop     es
    call    ZX0_Decode
    pop     es
    F_JC AC_bad
    mov     si,OFFSET decode_buf
    mov     cx,[block_decoded]
    mov     di,si
AC_write:
    mov     ax,[decoded_lo]
    mov     dx,[decoded_hi]
    add     ax,cx
    adc     dx,0
    cmp     dx,[expected_hi]
    F_JA AC_bad
    jb      short AC_size_ok
    cmp     ax,[expected_lo]
    F_JA AC_bad
AC_size_ok:
    mov     [write_len],cx
    mov     bx,[out_handle]
    mov     dx,di
    mov     ah,40h
    int     21h
    F_JC AC_bad
    cmp     ax,[write_len]
    F_JNE AC_bad
    mov     ax,[write_len]
    add     [decoded_lo],ax
    jnc     short AC_update_done
    inc     word ptr [decoded_hi]
AC_update_done:
    mov     word ptr [block_count],0
    mov     word ptr [block_target],0
    mov     si,[packet_cursor]
    mov     cx,[packet_left]
    jmp     AC_next
AC_ok:
    pop     di
    pop     si
    pop     dx
    pop     cx
    pop     bx
    pop     ax
    clc
    ret
AC_bad:
    pop     di
    pop     si
    pop     dx
    pop     cx
    pop     bx
    pop     ax
    stc
    ret
append_compressed ENDP

out_name DB 13 DUP(0)
metadata DB 64 DUP(0)
data_buf DB 512 DUP(0)
block_buf DB 8196 DUP(0)
decode_buf DB 4096 DUP(0)
out_handle DW 0FFFFh
file_time DW 0
file_date DW 0
file_attr DB 0
file_created DB 0
out_pos_lo DW 0
out_pos_hi DW 0
decoded_lo DW 0
decoded_hi DW 0
expected_lo DW 0
expected_hi DW 0
metadata_count DW 0
data_count DW 0
block_count DW 0
block_target DW 0
block_decoded DW 0
size_digit DB 0
name_dot DB 0
size_mul_hi DW 0
write_len DW 0
packet_cursor DW 0
packet_left DW 0
compressed_flag DB 0
ready_msg DB 'ZMODEM receive ready.',13,10,'$'
file_done DB 'Received file.',13,10,'$'
fail_msg DB 'ZMODEM receive failed.',13,10,'$'
IFDEF BUFFERED_RECEIVE
BW_stack DB 2048 DUP(0)
BW_stack_top LABEL BYTE
BW_image_end LABEL BYTE
ENDIF
END start
