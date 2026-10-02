; Binary stage reconstructed by the printable .COM loader at CS:6000h.
; COM1 only. All protocol bytes bypass DOS/CTTY console processing.
.286
.model tiny
.code
ORG 6000h
start:
    cld
    push    cs
    pop     ds
    mov     ax,0040h
    mov     es,ax
    mov     bp,word ptr es:[0]
    or      bp,bp
    jnz     short boot_have_uart
    jmp     boot_exit_fail
boot_have_uart:
    mov     ax,00E3h                 ; BIOS COM1: 9600, no parity, 8N1
    xor     dx,dx
    int     14h
    mov     dx,bp
    add     dx,3
    mov     al,083h
    out     dx,al
    mov     dx,bp
    mov     al,12                    ; enforce divisor even on older BIOSes
    out     dx,al
    inc     dx
    xor     al,al
    out     dx,al
    add     dx,2
    mov     al,3
    out     dx,al
    inc     dx
    out     dx,al                    ; DTR + RTS
    mov     dx,bp
    call    UART_GuardAcquire
    mov     bx,word ptr es:[006Ch]
boot_settle:
    mov     ax,word ptr es:[006Ch]
    sub     ax,bx
    cmp     ax,9
    jb      boot_settle
    mov     al,'B'
    call    boot_put
    mov     al,'1'
    call    boot_put
    mov     al,'>'
    call    boot_put
    call    boot_word
    dec     ax
    cmp     ax,8191
    ja      boot_fail
    inc     ax
    mov     cx,ax
    mov     di,2000h
    xor     si,si
boot_receive:
    call    boot_get
    mov     [di],al
    inc     di
    xor     ah,ah
    add     si,ax
    loop    boot_receive
    call    boot_word
    cmp     ax,si
    jne     boot_fail
    ; No more traffic until B2>; hand the original BIOS timer to stage2.
    call    UART_GuardRelease
    push    cs
    pop     es
    mov     ax,2000h
    jmp     ax
boot_fail:
    mov     al,'B'
    call    boot_put
    mov     al,'!'
    call    boot_put
boot_exit_fail:
    call    UART_GuardRelease
    mov     ax,4C01h
    int     21h

; AX = little-endian word. Byte I/O retains every other general register.
boot_word PROC NEAR
    push    bx
    call    boot_get
    mov     bl,al
    call    boot_get
    mov     ah,al
    mov     al,bl
    pop     bx
    ret
boot_word ENDP

; Fatal timeout abandons any active near-call frames and exits the process.
boot_get PROC NEAR
    push    bx
    push    dx
    mov     bx,word ptr es:[006Ch]
boot_get_wait:
    mov     dx,bp
    add     dx,5
    in      al,dx
    test    al,1
    jnz     short boot_get_ready
    mov     ax,word ptr es:[006Ch]
    sub     ax,bx
    cmp     ax,36
    jb      short boot_get_wait
    jmp     boot_fail
boot_get_ready:
    mov     dx,bp
    in      al,dx
    pop     dx
    pop     bx
    ret
boot_get ENDP

boot_put PROC NEAR
    push    ax
    push    bx
    push    dx
    mov     ah,al
    mov     bx,word ptr es:[006Ch]
boot_put_wait:
    mov     dx,bp
    add     dx,5
    in      al,dx
    test    al,020h
    jnz     short boot_put_ready
    push    ax
    mov     ax,word ptr es:[006Ch]
    sub     ax,bx
    cmp     ax,36
    pop     ax
    jb      short boot_put_wait
    jmp     boot_exit_fail
boot_put_ready:
    mov     dx,bp
    mov     al,ah
    out     dx,al
    pop     dx
    pop     bx
    pop     ax
    ret
boot_put ENDP
include uart_guard.inc

END start
