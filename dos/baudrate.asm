; Set the selected COM port baud through the UART divisor and notify the host before switching.
; Build with UASM as a 16-bit real-mode .COM image.
.286
.model tiny
.code
ORG 100h
start:
    jmp main

include serial.inc
include baud.asm

main:
    push cs
    pop ds
    mov si, 81h
    mov cl, byte ptr [80h]
    xor ch, ch
    or cx, cx
    jnz command_nonempty
    jmp usage
command_nonempty:
skip_leading:
    or cx, cx
    jnz skip_leading_check
    jmp usage
skip_leading_check:
    cmp byte ptr [si], ' '
    jne copy_rate
    inc si
    dec cx
    jmp short skip_leading
copy_rate:
    mov di, OFFSET rate_arg
    mov bx, 7
copy_rate_loop:
    jcxz rate_done
    lodsb
    cmp al, 0Dh
    je rate_done
    cmp al, ' '
    je trim_tail
    stosb
    dec bx
    jz usage
    dec cx
    jmp short copy_rate_loop
trim_tail:
    dec cx
    jcxz rate_done
    cmp byte ptr [si], ' '
    je trim_more
    jmp usage
trim_more:
    inc si
    jmp short trim_tail
rate_done:
    mov byte ptr [di], 0

    mov si, OFFSET rate_arg
    mov di, OFFSET baud_1200
    call string_equal
    jnc rate_1200
    mov si, OFFSET rate_arg
    mov di, OFFSET baud_2400
    call string_equal
    jnc rate_2400
    mov si, OFFSET rate_arg
    mov di, OFFSET baud_9600
    call string_equal
    jnc rate_9600
    mov si, OFFSET rate_arg
    mov di, OFFSET baud_19200
    call string_equal
    jnc rate_19200
    mov si, OFFSET rate_arg
    mov di, OFFSET baud_38400
    call string_equal
    jnc rate_38400
    mov si, OFFSET rate_arg
    mov di, OFFSET baud_57600
    call string_equal
    jnc rate_57600
    mov si, OFFSET rate_arg
    mov di, OFFSET baud_115200
    call string_equal
    jnc rate_115200
    jmp short usage

rate_1200:
    mov dx, 0
    mov ax, 1200
    jmp short set_rate
rate_2400:
    mov dx, 0
    mov ax, 2400
    jmp short set_rate
rate_9600:
    mov dx, 0
    mov ax, 9600
    jmp short set_rate
rate_19200:
    mov dx, 0
    mov ax, 19200
    jmp short set_rate
rate_38400:
    mov dx, 0
    mov ax, 38400
    jmp short set_rate
rate_57600:
    mov dx, 0
    mov ax, 57600
    jmp short set_rate
rate_115200:
    mov dx, 1
    mov ax, 0C200h
set_rate:
    call UART_Init
    jc uart_error
    call UART_SendBaudMarker
    jc uart_error
    call UART_RestoreBreak
    mov dx, OFFSET changed_msg
    mov ah, 09h
    int 21h
    mov ax, 4C00h
    int 21h

; DS:SI and DS:DI point to NUL-terminated strings. CF=0 if equal.
string_equal PROC NEAR
    push ax
    push si
    push di
compare_char:
    mov al, byte ptr [si]
    cmp al, byte ptr [di]
    jne strings_differ
    or al, al
    jz strings_equal
    inc si
    inc di
    jmp short compare_char
strings_equal:
    pop di
    pop si
    pop ax
    clc
    ret
strings_differ:
    pop di
    pop si
    pop ax
    stc
    ret
string_equal ENDP

usage:
    mov dx, OFFSET usage_msg
    mov ah, 09h
    int 21h
    mov ax, 4C01h
    int 21h
uart_error:
    call UART_Restore
    mov dx, OFFSET error_msg
    mov ah, 09h
    int 21h
    mov ax, 4C02h
    int 21h

rate_arg DB 8 DUP(0)
baud_1200 DB '1200',0
baud_2400 DB '2400',0
baud_9600 DB '9600',0
baud_19200 DB '19200',0
baud_38400 DB '38400',0
baud_57600 DB '57600',0
baud_115200 DB '115200',0
changed_msg DB 'Selected COM port baud changed.',13,10,'$'
usage_msg DB 'Usage: BAUDRATE 1200|2400|9600|19200|38400|57600|115200',13,10,'$'
error_msg DB 'Unable to switch selected COM port baud rate.',13,10,'$'
END start
