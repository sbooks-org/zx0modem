; DOS real-mode ZMODEM sender. UASM, 80286 instruction set only.
.286
B_JC MACRO target
    LOCAL skip
    jnc     short skip
    jmp     target
skip:
ENDM
B_JNE MACRO target
    LOCAL skip
    je      short skip
    jmp     target
skip:
ENDM
B_JE MACRO target
    LOCAL skip
    jne     short skip
    jmp     target
skip:
ENDM
B_JZ MACRO target
    LOCAL skip
    jnz     short skip
    jmp     target
skip:
ENDM
B_JA MACRO target
    LOCAL skip
    jbe     short skip
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

main PROC NEAR
    cld
    push    cs
    pop     ds
    ; Parse a single DOS command-tail pathname (spaces are not accepted).
    mov     si,80h
    xor     cx,cx
    mov     cl,[si]
    or      cx,cx
    jnz     short got_args
    jmp     send_usage
got_args:
    inc     si
skip_spaces:
    cmp     byte ptr [si],' '
    jne     got_path
    inc     si
    loop    skip_spaces
    jmp     send_usage
got_path:
    mov     di,OFFSET pathbuf
    mov     bx,64
copy_path:
    jcxz    path_done
    lodsb
    cmp     al,' '
    je      path_done
    cmp     al,0Dh
    je      path_done
    stosb
    dec     bx
B_JZ send_usage
    loop    copy_path
path_done:
    mov     al,0
    stosb
    mov     dx,OFFSET pathbuf
    mov     ax,3D00h
    int     21h
B_JC send_error
    mov     [file_handle],ax
    mov     bx,ax
    mov     ax,5700h
    int     21h
B_JC close_error
    call    MD_Validate
B_JC close_error
    mov     [file_time],cx
    mov     [file_date],dx
    mov     dx,OFFSET pathbuf
    mov     ax,4300h
    int     21h
B_JC close_error
    and     cl,1
    mov     [file_attr],cl
    mov     ax,4202h
    xor     cx,cx
    xor     dx,dx
    int     21h
B_JC close_error
    mov     [file_size_lo],ax
    mov     [file_size_hi],dx
    mov     ax,4200h
    xor     cx,cx
    xor     dx,dx
    int     21h
B_JC close_error
    call    UART_Init
B_JC close_error
    xor     dx,dx
    mov     ax,UART_BAUD_9600
    call    UART_SetBaud
B_JC close_error
    call    UART_FlushInput
B_JC close_error
    call    wait_receiver
B_JC close_error

    ; Send the real basename and decimal original size in ZFILE metadata.
    call    build_metadata
B_JC close_error
    mov     byte ptr [zm_type],ZFILE
    mov     word ptr [zm_pos_lo],0
    mov     word ptr [zm_pos_hi],0
    call    ZM_MakeHeader
    mov     si,OFFSET zm_header
    call    ZM_SendBinHeader
B_JC close_error
    mov     si,OFFSET metadata
    mov     cx,[metadata_len]
    mov     al,ZCRCW
    call    ZM_SendData16
B_JC close_error
    call    ZM_RecvHeader
B_JC close_error
    cmp     byte ptr [zm_header],ZSKIP
B_JE send_skip
    cmp     byte ptr [zm_header],ZRPOS
B_JNE close_error
    jmp     send_next

send_skip:
    call    UART_Restore
    mov     bx,[file_handle]
    mov     ah,3Eh
    int     21h
    mov     dx,OFFSET skip_msg
    mov     ah,09h
    int     21h
    mov     ax,4C00h
    int     21h
send_next:
    mov     bx,[file_handle]
    mov     ah,3Fh
    mov     cx,512
    mov     dx,OFFSET data_buf
    int     21h
B_JC close_error
    or      ax,ax
B_JZ send_eof
    mov     [data_count],ax
    mov     byte ptr [zm_type],ZDATA
    mov     ax,[file_pos_lo]
    mov     [zm_pos_lo],ax
    mov     ax,[file_pos_hi]
    mov     [zm_pos_hi],ax
    call    ZM_MakeHeader
    mov     si,OFFSET zm_header
    call    ZM_SendBinHeader
B_JC close_error
    mov     si,OFFSET data_buf
    mov     cx,[data_count]
    mov     al,ZCRCW
    call    ZM_SendData16
B_JC close_error
    mov     ax,[data_count]
    add     [file_pos_lo],ax
    jnc     short pos_ok
    inc     word ptr [file_pos_hi]
pos_ok:
    call    ZM_RecvHeader
B_JC close_error
    cmp     byte ptr [zm_header],ZACK
B_JE send_next
    cmp     byte ptr [zm_header],ZRPOS
B_JNE close_error
    ; Restart only inside this transfer; seek to peer's requested offset.
    mov     ax,word ptr [zm_header+1]
    mov     [file_pos_lo],ax
    mov     ax,word ptr [zm_header+3]
    mov     [file_pos_hi],ax
    mov     bx,[file_handle]
    mov     cx,[file_pos_hi]
    mov     dx,[file_pos_lo]
    mov     ax,4200h
    int     21h
B_JC close_error
    jmp     send_next

send_eof:
    mov     byte ptr [zm_type],ZEOF
    mov     ax,[file_pos_lo]
    mov     [zm_pos_lo],ax
    mov     ax,[file_pos_hi]
    mov     [zm_pos_hi],ax
    call    ZM_MakeHeader
    mov     si,OFFSET zm_header
    call    ZM_SendBinHeader
B_JC close_error
    call    ZM_RecvHeader
B_JC close_error
    cmp     byte ptr [zm_header],ZRINIT
B_JNE close_error
    mov     byte ptr [zm_type],ZFIN
    mov     word ptr [zm_pos_lo],0
    mov     word ptr [zm_pos_hi],0
    call    ZM_MakeHeader
    mov     si,OFFSET zm_header
    call    ZM_SendHexHeader
B_JC close_error
    call    ZM_RecvHeader
B_JC close_error
    cmp     byte ptr [zm_header],ZFIN
B_JNE close_error
    mov     al,'O'
    call    UART_PutChar
    mov     al,'O'
    call    UART_PutChar
    call    UART_Restore
    mov     bx,[file_handle]
    mov     ah,3Eh
    int     21h
    mov     dx,OFFSET done_msg
    mov     ah,09h
    int     21h
    mov     ax,4C00h
    int     21h

main ENDP

wait_receiver PROC NEAR
wr_send_init:
    mov     byte ptr [zm_type],ZRQINIT
    mov     word ptr [zm_pos_lo],0
    mov     word ptr [zm_pos_hi],0
    call    ZM_MakeHeader
    mov     si,OFFSET zm_header
    call    ZM_SendHexHeader
    jc      short wr_bad
wr_wait:
    call    ZM_RecvHeader
    jc      short wr_retry
    cmp     byte ptr [zm_header],ZRINIT
    je      short wr_good
    jmp     short wr_send_init
wr_retry:
    call    UART_PollCancel
    jc      short wr_bad
    jmp     short wr_send_init
wr_good:
    clc
    ret
wr_bad:
    stc
    ret
wait_receiver ENDP

build_metadata PROC NEAR
    push    ax
    push    bx
    push    cx
    push    dx
    push    si
    push    di
    push    bp
    mov     si,OFFSET pathbuf
    mov     [name_start],si
BM_find_base:
    lodsb
    or      al,al
    jz      short BM_base_found
    cmp     al,'/'
    je      short BM_new_base
    cmp     al,05Ch
    jne     short BM_find_base
BM_new_base:
    mov     [name_start],si
    jmp     short BM_find_base
BM_base_found:
    mov     si,[name_start]
    mov     di,OFFSET metadata
    xor     bx,bx
    xor     bp,bp
    mov     byte ptr [name_dot],0
BM_name:
    lodsb
    or      al,al
    jz      short BM_name_end
    cmp     al,'.'
    jne     short BM_name_char
    cmp     byte ptr [name_dot],0
B_JNE BM_bad
    or      bx,bx
B_JZ BM_bad
    mov     byte ptr [name_dot],1
    mov     al,'.'
    stosb
    jmp     short BM_name
BM_name_char:
    cmp     al,'A'
    jb      short BM_lower
    cmp     al,'Z'
    jbe     short BM_store
BM_lower:
    cmp     al,'a'
    jb      short BM_digit
    cmp     al,'z'
    ja      short BM_digit
    and     al,0DFh
    jmp     short BM_store
BM_digit:
    cmp     al,'0'
    jb      short BM_punct
    cmp     al,'9'
    jbe     short BM_store
BM_punct:
    cmp     al,'_'
    je      short BM_store
    cmp     al,'-'
B_JNE BM_bad
BM_store:
    cmp     byte ptr [name_dot],0
    jne     short BM_ext
    inc     bx
    cmp     bx,8
B_JA BM_bad
    jmp     short BM_copy
BM_ext:
    inc     bp
    cmp     bp,3
B_JA BM_bad
BM_copy:
    stosb
    jmp     short BM_name
BM_name_end:
    or      bx,bx
B_JZ BM_bad
    cmp     byte ptr [name_dot],0
    je      short BM_name_term
    or      bp,bp
B_JZ BM_bad
BM_name_term:
    xor     al,al
    stosb
    xor     bp,bp
    mov     ax,[file_size_lo]
    mov     dx,[file_size_hi]
    or      ax,dx
    jnz     short BM_size_loop
    mov     al,'0'
    stosb
    jmp     short BM_size_done
BM_size_loop:
    mov     cx,ax
    mov     ax,dx
    xor     dx,dx
    mov     bx,10
    div     bx
    mov     [size_quot_hi],ax
    mov     ax,cx
    div     bx
    mov     [size_quot_lo],ax
    mov     [reverse_digits+bp],dl
    inc     bp
    mov     dx,[size_quot_hi]
    mov     ax,[size_quot_lo]
    or      ax,dx
    jnz     short BM_size_loop
    mov     cx,bp
BM_write_digit:
    dec     bp
    mov     al,[reverse_digits+bp]
    add     al,'0'
    stosb
    loop    BM_write_digit
BM_size_done:
    xor     al,al
    stosb
    mov     al,1
    stosb
    mov     al,[file_attr]
    stosb
    mov     al,1
    stosb
    xor     al,al
    stosb
    mov     ax,[file_time]
    stosw
    mov     ax,[file_date]
    stosw
    mov     ax,di
    sub     ax,OFFSET metadata
    mov     [metadata_len],ax
    pop     bp
    pop     di
    pop     si
    pop     dx
    pop     cx
    pop     bx
    pop     ax
    clc
    ret
BM_bad:
    pop     bp
    pop     di
    pop     si
    pop     dx
    pop     cx
    pop     bx
    pop     ax
    stc
    ret
build_metadata ENDP

send_usage:
    mov     dx,OFFSET usage
    mov     ah,09h
    int     21h
    mov     ax,4C01h
    int     21h
send_error:
    mov     dx,OFFSET open_error
    mov     ah,09h
    int     21h
    mov     ax,4C01h
    int     21h
close_error:
    call    UART_Restore
    mov     bx,[file_handle]
    mov     ah,3Eh
    int     21h
    mov     dx,OFFSET fail_msg
    mov     ah,09h
    int     21h
    mov     ax,4C01h
    int     21h

; Metadata filename NUL decimal size NUL version/readonly/compression/time/date.
metadata DB 64 DUP(0)
pathbuf DB 64 DUP(0)
file_handle DW 0FFFFh
file_time DW 0
file_date DW 0
file_attr DB 0
file_size_lo DW 0
file_size_hi DW 0
file_pos_lo DW 0
file_pos_hi DW 0
data_count DW 0
data_buf DB 512 DUP(0)
metadata_len DW 0
name_start DW 0
name_dot DB 0
reverse_digits DB 10 DUP(0)
size_quot_hi DW 0
size_quot_lo DW 0
usage DB 'Usage: ZSEND file',13,10,'$'
open_error DB 'Cannot open source file.',13,10,'$'
fail_msg DB 'ZMODEM send failed.',13,10,'$'
done_msg DB 'ZMODEM send complete.',13,10,'$'
skip_msg DB 'Remote skipped the file.',13,10,'$'
END start
