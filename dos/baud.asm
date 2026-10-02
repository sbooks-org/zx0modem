.286

; Include after serial.inc in a single-segment .COM program. This module never
; sends anything except the explicit baud-change marker requested by caller.
; UART_SendBaudMarker takes the new numeric rate in DX:AX, emits
; ESC ] 777 ; BAUD ; decimal-rate BEL at the current rate, waits until TEMT,
; leaves a 250ms+ guard interval, then changes the UART divisor. CF reports an
; unsupported rate, cancellation, or output-drain timeout.

UART_SendBaudMarker PROC NEAR
    push ax
    push bx
    push cx
    push dx
    push si
    push di
    push bp
    push ds
    push es
    ; Validate and choose canonical decimal spelling before emitting anything.
    or dx, dx
    jnz UB_115200
    cmp ax, UART_BAUD_1200
    jne UB_2400
    mov si, OFFSET BAUD_RATE_1200
    jmp short UART_Baud_RateReady
UB_2400:
    cmp ax, UART_BAUD_2400
    jne UB_9600
    mov si, OFFSET BAUD_RATE_2400
    jmp short UART_Baud_RateReady
UB_9600:
    cmp ax, UART_BAUD_9600
    jne UB_19200
    mov si, OFFSET BAUD_RATE_9600
    jmp short UART_Baud_RateReady
UB_19200:
    cmp ax, UART_BAUD_19200
    jne UB_38400
    mov si, OFFSET BAUD_RATE_19200
    jmp short UART_Baud_RateReady
UB_38400:
    cmp ax, UART_BAUD_38400
    jne UB_57600
    mov si, OFFSET BAUD_RATE_38400
    jmp short UART_Baud_RateReady
UB_57600:
    cmp ax, UART_BAUD_57600
    je UART_Baud_Select57600
    jmp UART_Baud_Fail
UART_Baud_Select57600:
    mov si, OFFSET BAUD_RATE_57600
    jmp short UART_Baud_RateReady
UB_115200:
    cmp dx, 1
    je UART_Baud_Check115200Low
    jmp UART_Baud_Fail
UART_Baud_Check115200Low:
    cmp ax, 49664
    je UART_Baud_Select115200
    jmp UART_Baud_Fail
UART_Baud_Select115200:
    mov si, OFFSET BAUD_RATE_115200
UART_Baud_RateReady:
    mov word ptr cs:[BAUD_TARGET_LO], ax
    mov word ptr cs:[BAUD_TARGET_HI], dx
    mov di, si
    mov si, OFFSET BAUD_MARKER_PREFIX
UART_Baud_Prefix:
    mov al, byte ptr cs:[si]
    inc si
    or al, al
    jz UART_Baud_Rate
    jnc UART_Baud_PrefixOk
    jmp UART_Baud_Fail
UART_Baud_PrefixOk:
    jmp short UART_Baud_Prefix
UART_Baud_Rate:
    mov al, byte ptr cs:[di]
    inc di
    or al, al
    jz UART_Baud_Bell
    call UART_PutChar
    jnc UART_Baud_RateOk
    jmp UART_Baud_Fail
UART_Baud_RateOk:
    jmp short UART_Baud_Rate
UART_Baud_Bell:
    mov al, 007h
    call UART_PutChar
    jnc UART_Baud_BellOk
    jmp UART_Baud_Fail
UART_Baud_BellOk:

    ; Wait for the final BEL to leave the shift register, not merely THR.
    call UART_ReadTicks
    mov si, ax
UART_Baud_Temt:
    call UART_PollCancel
    jnc UART_Baud_TemtContinue
    jmp UART_Baud_Fail
UART_Baud_TemtContinue:
    mov dx, word ptr cs:[UART_BASE]
    or dx, dx
    jnz UART_Baud_HavePort
    jmp UART_Baud_Fail
UART_Baud_HavePort:
    add dx, 5
    in al, dx
    test al, 040h
    jnz UART_Baud_Delay
    call UART_ReadTicks
    sub ax, si
    cmp ax, 08000h
    jae UART_Baud_Temt
    cmp ax, UART_TIMEOUT_TICKS
    jb UART_Baud_Temt
    jmp short UART_Baud_Fail

UART_Baud_Delay:
    call UART_ReadTicks
    mov si, ax
UART_Baud_DelayLoop:
    call UART_ReadTicks
    sub ax, si
    cmp ax, 08000h
    jae UART_Baud_DelayLoop
    cmp ax, 5
    jb UART_Baud_DelayLoop
    mov ax, word ptr cs:[BAUD_TARGET_LO]
    mov dx, word ptr cs:[BAUD_TARGET_HI]
    call UART_SetBaud
    jmp short UART_Baud_Done
UART_Baud_Fail:
    stc
UART_Baud_Done:
    pop es
    pop ds
    pop bp
    pop di
    pop si
    pop dx
    pop cx
    pop bx
    pop ax
    ret
UART_SendBaudMarker ENDP

BAUD_MARKER_PREFIX DB 01Bh, ']', '777;BAUD;', 0
BAUD_RATE_1200     DB '1200', 0
BAUD_RATE_2400     DB '2400', 0
BAUD_RATE_9600     DB '9600', 0
BAUD_RATE_19200    DB '19200', 0
BAUD_RATE_38400    DB '38400', 0
BAUD_RATE_57600    DB '57600', 0
BAUD_RATE_115200   DB '115200', 0
BAUD_TARGET_LO     DW 0
BAUD_TARGET_HI     DW 0
