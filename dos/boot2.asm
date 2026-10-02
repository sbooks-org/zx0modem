; Binary installer received at CS:2000h by BOOT1. No DOS console I/O.
; Requires DOS 3+ (create-new), one complete 64K COM segment, COM1 at 9600.
.286
B_JC MACRO target
    LOCAL skip
    jnc     short skip
    jmp     target
skip:
ENDM
B_JE MACRO target
    LOCAL skip
    jne     short skip
    jmp     target
skip:
ENDM
B_JNE MACRO target
    LOCAL skip
    je      short skip
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
ORG 2000h
; Buffers occupy memory only, never bytes in the transmitted image.
BOOT_ENCODED EQU 9000h                ; 8192 bytes, ends at AFFFh
BOOT_DECODED EQU 0B000h               ; 4096 bytes, ends at BFFFh
start:
    jmp     boot_main

include serial.inc
include zx0.inc

boot_main PROC NEAR
    cld
    push    cs
    pop     ds
    push    cs
    pop     es
    ; Stage1 already selected COM1, established 9600 and asserted DTR/RTS.
    ; Init leaves that divisor alone and hooks local Ctrl-Break cancellation.
    call    UART_Init
    B_JC boot_error
    ; Finish potentially slow disk creation BEFORE inviting UART traffic.
    mov     dx,OFFSET boot_name
    xor     cx,cx
    mov     ah,5Bh                   ; create NEW: never truncate an old file
    int     21h
    B_JC boot_error
    mov     [boot_handle],ax
    mov     byte ptr [boot_created],1
    mov     si,OFFSET boot_ready
    call    boot_message
    B_JC boot_error
    call    boot_read_word
    B_JC boot_error
    or      ax,ax
    B_JE boot_error
    cmp     ax,60000
    B_JA boot_error
    mov     [boot_remaining],ax
boot_block:
    call    boot_read_word
    B_JC boot_error
    or      ax,ax
    B_JE boot_error
    cmp     ax,ZX0_MAX_BLOCK
    B_JA boot_error
    cmp     ax,[boot_remaining]
    B_JA boot_error
    mov     [boot_decoded_len],ax
    call    boot_read_word
    B_JC boot_error
    mov     [boot_stored_flags],ax
    and     ax,7FFFh
    B_JE boot_error
    cmp     ax,8192
    B_JA boot_error
    mov     [boot_stored_len],ax
    test    word ptr [boot_stored_flags],8000h
    jz      short boot_length_ok
    cmp     ax,[boot_decoded_len]
    B_JNE boot_error
boot_length_ok:
    mov     cx,ax
    mov     di,BOOT_ENCODED
    xor     bp,bp
boot_read_bytes:
    call    UART_GetChar
    B_JC boot_error
    mov     [di],al
    inc     di
    xor     ah,ah
    add     bp,ax                    ; wrapping sum of STORED, not decoded bytes
    loop    boot_read_bytes
    call    boot_read_word
    B_JC boot_error
    cmp     ax,bp
    B_JNE boot_error
    mov     dx,BOOT_ENCODED
    test    word ptr [boot_stored_flags],8000h
    jnz     short boot_write
    push    cs
    pop     es
    cld
    mov     si,BOOT_ENCODED
    mov     cx,[boot_stored_len]
    mov     di,BOOT_DECODED
    mov     bp,[boot_decoded_len]
    call    ZX0_Decode
    B_JC boot_error
    mov     dx,BOOT_DECODED
boot_write:
    mov     bx,[boot_handle]
    mov     cx,[boot_decoded_len]
    mov     ah,40h
    int     21h
    B_JC boot_error
    cmp     ax,[boot_decoded_len]
    B_JNE boot_error                 ; short write, including disk full
    sub     [boot_remaining],ax
    mov     al,'+'
    call    UART_PutChar
    B_JC boot_error
    cmp     word ptr [boot_remaining],0
    B_JNE boot_block
    mov     bx,[boot_handle]
    mov     ah,3Eh
    int     21h
    B_JC boot_error
    mov     word ptr [boot_handle],0FFFFh
    mov     byte ptr [boot_created],0 ; complete file must survive marker failure
    mov     si,OFFSET boot_done
    call    boot_message
    B_JC boot_error
    call    UART_RestoreBreak        ; return console ownership after B3>
    mov     ax,4C00h
    int     21h
boot_error:
    cmp     word ptr [boot_handle],0FFFFh
    je      short boot_error_delete
    mov     bx,[boot_handle]
    mov     ah,3Eh
    int     21h
    mov     word ptr [boot_handle],0FFFFh
boot_error_delete:
    cmp     byte ptr [boot_created],0
    je      short boot_error_signal
    mov     dx,OFFSET boot_name
    mov     ah,41h
    int     21h                     ; delete ONLY the file this run created
boot_error_signal:
    mov     byte ptr cs:[UART_CANCEL],0
    mov     si,OFFSET boot_failed
    call    boot_message             ; best effort if the link itself failed
    call    UART_RestoreBreak
    mov     ax,4C01h
    int     21h
boot_main ENDP

; AX = little-endian word, carry on timeout/cancellation, BX retained.
boot_read_word PROC NEAR
    push    bx
    call    UART_GetChar
    jc      short boot_word_done
    mov     bl,al
    call    UART_GetChar
    jc      short boot_word_done
    mov     ah,al
    mov     al,bl
boot_word_done:
    pop     bx
    ret
boot_read_word ENDP

; Send a NUL-terminated marker directly, never through DOS or serial BIOS.
boot_message PROC NEAR
    lodsb
    or      al,al
    jz      short boot_message_done
    call    UART_PutChar
    jc      short boot_message_done
    jmp     short boot_message
boot_message_done:
    ret
boot_message ENDP

boot_handle       DW 0FFFFh
boot_created      DB 0
boot_remaining    DW 0
boot_decoded_len  DW 0
boot_stored_flags DW 0
boot_stored_len   DW 0
boot_name         DB 'ZRECV.COM',0
boot_ready        DB 'B2>',0
boot_done         DB 'B3>',0
boot_failed       DB 'B!',0
END start
