.286

UART_TIMEOUT_TICKS EQU 18             ; one BIOS second (18.2 ticks)
UART_BAUD_1200     EQU 1200
UART_BAUD_2400     EQU 2400
UART_BAUD_9600     EQU 9600
UART_BAUD_19200    EQU 19200
UART_BAUD_38400    EQU 38400
UART_BAUD_57600    EQU 57600
UART_BAUD_115200   EQU 115200

; Include into a single-segment .COM program. All state references use CS so
; serial routines remain valid while DOS temporarily changes DS.
; ABI: near calls. All routines preserve general registers and segments.
; UART_Init: CF=1 if no COM port; otherwise saves state and sets 8N1, DTR/RTS.
; UART_GetChar: AL=received byte; CF=1 on timeout or local cancellation.
; UART_PutChar: AL preserved; CF=1 on CTS/TX timeout or cancellation.
; UART_FlushInput: discards received bytes; CF=1 on local cancellation.
; UART_SetBaud: DX:AX=one supported numeric rate; CF=1 if unsupported/no UART.
; UART_PollCancel: consumes ESC, Ctrl-C, or Ctrl-Break; CF=1 iff consumed.
; UART_Restore: restores original divisor/LCR/MCR. No bytes are transmitted by
; any routine in this include.

UART_Init PROC NEAR
    push ax
    push bx
    push cx
    push dx
    push si
    push di
    push bp
    push ds
    push es
    mov word ptr cs:[UART_BASE], 0
    mov byte ptr cs:[UART_PORT_NUM], 0
    mov ax, 0040h
    mov ds, ax
    mov si, 0000h
    mov cx, 4
UART_Init_Find:
    mov dx, word ptr [si]
    or dx, dx
    jnz UART_Init_Found
    add si, 2
    loop UART_Init_Find
    pop es
    pop ds
    pop bp
    pop di
    pop si
    pop dx
    pop cx
    pop bx
    pop ax
    stc
    ret
UART_Init_Found:
    mov word ptr cs:[UART_BASE], dx
    mov ax, si
    shr ax, 1
    mov byte ptr cs:[UART_PORT_NUM], al
    mov dx, word ptr cs:[UART_BASE]
    add dx, 3
    in al, dx
    mov byte ptr cs:[UART_OLD_LCR], al
    and al, 07Fh
    out dx, al
    mov dx, word ptr cs:[UART_BASE]
    call UART_GuardAcquire
    mov dx, word ptr cs:[UART_BASE]
    add dx, 4
    in al, dx
    mov byte ptr cs:[UART_OLD_MCR], al
    ; Save current divisor while preserving the line-control setting.
    mov dx, word ptr cs:[UART_BASE]
    add dx, 3
    mov al, byte ptr cs:[UART_OLD_LCR]
    or al, 080h
    out dx, al
    mov dx, word ptr cs:[UART_BASE]
    in al, dx
    mov byte ptr cs:[UART_OLD_DLL], al
    inc dx
    in al, dx
    mov byte ptr cs:[UART_OLD_DLM], al
    mov dx, word ptr cs:[UART_BASE]
    add dx, 3
    mov al, byte ptr cs:[UART_OLD_LCR]
    and al, 07Fh
    out dx, al
    mov dx, word ptr cs:[UART_BASE]
    add dx, 4
    mov al, byte ptr cs:[UART_OLD_MCR]
    or al, 003h                         ; assert DTR and RTS for flow control
    out dx, al
    mov dx, word ptr cs:[UART_BASE]
    add dx, 3
    mov al, 003h                         ; 8 data bits, no parity, one stop
    out dx, al
    ; Leave the existing divisor unchanged; callers choose the link rate.
    ; Convert Ctrl-Break (INT 1Bh) into a deferred cancellation signal.
    mov ax, 351Bh
    int 21h
    mov word ptr cs:[UART_OLD_BREAK_OFF], bx
    mov word ptr cs:[UART_OLD_BREAK_SEG], es
    push ds
    push cs
    pop ds
    mov dx, OFFSET UART_CtrlBreakHandler
    mov ax, 251Bh
    int 21h
    pop ds
    mov byte ptr cs:[UART_CANCEL], 0
    mov byte ptr cs:[UART_BREAK_HOOK], 1
    pop es
    pop ds
    pop bp
    pop di
    pop si
    pop dx
    pop cx
    pop bx
    pop ax
    clc
    ret
UART_Init ENDP

UART_Restore PROC NEAR
    push cx
    push si
    push di
    push bp
    push ds
    push es
    push ax
    push dx
    push bx
    pushf
    cli
    mov bx, word ptr cs:[UART_BASE]
    or bx, bx
    jz UART_Restore_Done
    mov dx, bx
    add dx, 3
    mov al, byte ptr cs:[UART_OLD_LCR]
    or al, 080h
    out dx, al
    mov dx, bx
    mov al, byte ptr cs:[UART_OLD_DLL]
    out dx, al
    inc dx
    mov al, byte ptr cs:[UART_OLD_DLM]
    out dx, al
    mov dx, bx
    add dx, 3
    mov al, byte ptr cs:[UART_OLD_LCR]
    out dx, al
    mov dx, bx
    add dx, 4
    mov al, byte ptr cs:[UART_OLD_MCR]
    out dx, al
    call UART_GuardRelease
    mov word ptr cs:[UART_BASE], 0
UART_Restore_Done:
    popf
    cmp byte ptr cs:[UART_BREAK_HOOK], 0
    je UART_Restore_NoHook
    push ds
    mov ax, word ptr cs:[UART_OLD_BREAK_SEG]
    mov ds, ax
    mov dx, word ptr cs:[UART_OLD_BREAK_OFF]
    mov ax, 251Bh
    int 21h
    pop ds
    mov byte ptr cs:[UART_BREAK_HOOK], 0
UART_Restore_NoHook:
    pop bx
    pop dx
    pop ax
    pop es
    pop ds
    pop bp
    pop di
    pop si
    pop cx
    clc
    ret
UART_Restore ENDP

; Release the console/IRQ0 guard and INT 1Bh, retaining the selected baud.
UART_RestoreBreak PROC NEAR
    push ax
    push bx
    push cx
    push si
    push di
    push bp
    push es
    push dx
    push ds
    call UART_GuardRelease
    cmp byte ptr cs:[UART_BREAK_HOOK], 0
    je UART_RestoreBreak_Done
    mov ax, word ptr cs:[UART_OLD_BREAK_SEG]
    mov ds, ax
    mov dx, word ptr cs:[UART_OLD_BREAK_OFF]
    mov ax, 251Bh
    int 21h
    mov byte ptr cs:[UART_BREAK_HOOK], 0
UART_RestoreBreak_Done:
    pop ds
    pop dx
    pop es
    pop bp
    pop di
    pop si
    pop cx
    pop bx
    pop ax
    ret
UART_RestoreBreak ENDP

; DX:AX numeric rate; accepted divisors derive from the 1.8432 MHz clock.
; The only supported rate requiring a nonzero high word is 115200.
UART_SetBaud PROC NEAR
    push ax
    push bx
    push dx
    push cx
    push si
    push di
    push bp
    push ds
    push es
    pushf
    mov byte ptr cs:[UART_BIOS_RATE], 0
    cmp word ptr cs:[UART_BASE], 0
    je UART_SetBaud_Bad
    or dx, dx
    jnz UART_SetBaud_Check115200
    cmp ax, UART_BAUD_1200
    jne UART_SetBaud_2400
    mov bx, 0060h
    mov byte ptr cs:[UART_BIOS_RATE], 083h
    jmp short UART_SetBaud_Write
UART_SetBaud_2400:
    cmp ax, UART_BAUD_2400
    jne UART_SetBaud_9600
    mov bx, 0030h
    mov byte ptr cs:[UART_BIOS_RATE], 0A3h
    jmp short UART_SetBaud_Write
UART_SetBaud_9600:
    cmp ax, UART_BAUD_9600
    jne UART_SetBaud_19200
    mov bx, 000Ch
    mov byte ptr cs:[UART_BIOS_RATE], 0E3h
    jmp short UART_SetBaud_Write
UART_SetBaud_19200:
    cmp ax, UART_BAUD_19200
    jne UART_SetBaud_38400
    mov bx, 0006h
    jmp short UART_SetBaud_Write
UART_SetBaud_38400:
    cmp ax, UART_BAUD_38400
    jne UART_SetBaud_57600
    mov bx, 0003h
    jmp short UART_SetBaud_Write
UART_SetBaud_57600:
    cmp ax, UART_BAUD_57600
    jne UART_SetBaud_Bad
    mov bx, 0002h
    jmp short UART_SetBaud_Write
UART_SetBaud_Check115200:
    cmp dx, 1
    jne UART_SetBaud_Bad
    cmp ax, 49664
    jne UART_SetBaud_Bad
    mov bx, 0001h
UART_SetBaud_Write:
    cmp byte ptr cs:[UART_BIOS_RATE], 0
    je UART_SetBaud_Direct
    push ax
    push bx
    push dx
    mov al, byte ptr cs:[UART_BIOS_RATE]
    mov ah, 0
    xor dx, dx
    mov dl, byte ptr cs:[UART_PORT_NUM]
    int 14h
    pop dx
    pop bx
    pop ax
UART_SetBaud_Direct:
    cli
    mov dx, word ptr cs:[UART_BASE]
    add dx, 3
    mov al, 003h
    or al, 080h
    out dx, al
    mov dx, word ptr cs:[UART_BASE]
    mov al, bl
    out dx, al
    inc dx
    xor al, al
    out dx, al
    mov dx, word ptr cs:[UART_BASE]
    add dx, 3
    mov al, 003h
    out dx, al
    mov dx, word ptr cs:[UART_BASE]
    inc dx
    xor al, al
    out dx, al                         ; BIOS initialization may enable UART IRQs
    mov dx, word ptr cs:[UART_BASE]
    add dx, 4
    mov al, 003h
    out dx, al
    popf
    pop es
    pop ds
    pop bp
    pop di
    pop si
    pop cx
    pop dx
    pop bx
    pop ax
    clc
    ret
UART_SetBaud_Bad:
    popf
    pop es
    pop ds
    pop bp
    pop di
    pop si
    pop cx
    pop dx
    pop bx
    pop ax
    stc
    ret
UART_SetBaud ENDP

; Nonblocking keyboard check. Consume ESC, Ctrl-C, or Ctrl-Break.
UART_PollCancel PROC NEAR
    push ax
    push bx
    push cx
    push dx
    push si
    push di
    push bp
    push ds
    push es
    cmp byte ptr cs:[UART_CANCEL], 0
    jne UART_PollCancel_Yes
    ; INT 16h can itself poll a serial console and consume protocol bytes.
    ; Inspect only keys already queued by the physical keyboard interrupt.
    pushf
    cli
    mov ax, 0040h
    mov es, ax
    mov bx, word ptr es:[001Ah]
    cmp bx, word ptr es:[001Ch]
    je UART_PollCancel_Empty
    mov ax, word ptr es:[bx]
    add bx, 2
    cmp bx, word ptr es:[0082h]
    jb UART_PollCancel_Advance
    mov bx, word ptr es:[0080h]
UART_PollCancel_Advance:
    mov word ptr es:[001Ah], bx
    popf
    cmp al, 01Bh
    je UART_PollCancel_Yes
    cmp al, 003h
    je UART_PollCancel_Yes
    cmp ax, 4600h                       ; BIOS Ctrl-Break key code
    je UART_PollCancel_Yes
    jmp short UART_PollCancel_No
UART_PollCancel_Empty:
    popf
UART_PollCancel_No:
    pop es
    pop ds
    pop bp
    pop di
    pop si
    pop dx
    pop cx
    pop bx
    pop ax
    clc
    ret
UART_PollCancel_Yes:
IFDEF EXPERIMENTAL_BAUD
    ; Training cleanup may poll again after a nested UART call consumed a key.
    mov byte ptr cs:[UART_CANCEL], 1
ENDIF
    pop es
    pop ds
    pop bp
    pop di
    pop si
    pop dx
    pop cx
    pop bx
    pop ax
    stc
    ret
UART_PollCancel ENDP

; Return the BIOS tick low word in AX. A word read is atomic on the 286.
UART_ReadTicks PROC NEAR
    push es
    mov ax, 0040h
    mov es, ax
    mov ax, word ptr es:[006Ch]
    pop es
    ret
UART_ReadTicks ENDP

UART_GetChar PROC NEAR
IFDEF EXPERIMENTAL_BAUD
    cmp byte ptr cs:[BT_probe_active],0
    je short UART_GetChar_Normal
    jmp BT_GetByte
UART_GetChar_Normal:
ENDIF
    push bx
    push cx
    push dx
    push si
    push di
    push bp
    push ds
    push es
    call UART_ReadTicks
    mov si, ax
UART_GetChar_Loop:
    call UART_PollCancel
    jc UART_GetChar_Fail
    mov dx, word ptr cs:[UART_BASE]
    or dx, dx
    jz UART_GetChar_Fail
    add dx, 5
    in al, dx
    test al, 001h
    jnz UART_GetChar_Ready
    call UART_ReadTicks
    sub ax, si
IFNDEF EXPERIMENTAL_BAUD
    cmp ax, 08000h
    jae UART_GetChar_Loop
ENDIF
    cmp ax, UART_TIMEOUT_TICKS
    jb UART_GetChar_Loop
    jmp short UART_GetChar_Fail
UART_GetChar_Ready:
    mov dx, word ptr cs:[UART_BASE]
    in al, dx
    clc
    jmp short UART_GetChar_Done
UART_GetChar_Fail:
    stc
UART_GetChar_Done:
    pop es
    pop ds
    pop bp
    pop di
    pop si
    pop dx
    pop cx
    pop bx
    ret
UART_GetChar ENDP

UART_PutChar PROC NEAR
    push bx
    push cx
    push dx
    push si
    push di
    push bp
    push ds
    push es
    push ax
    mov bl, al
    call UART_ReadTicks
    mov si, ax
UART_PutChar_Loop:
    call UART_PollCancel
    jc UART_PutChar_Fail
    mov dx, word ptr cs:[UART_BASE]
    or dx, dx
    jz UART_PutChar_Fail
    mov bp, dx
    add dx, 6
    in al, dx
    test al, 010h                       ; CTS must be asserted
    jz UART_PutChar_Wait
    mov dx, bp
    add dx, 5
    in al, dx
    test al, 020h                       ; transmitter holding register empty
    jnz UART_PutChar_Write
UART_PutChar_Wait:
    call UART_ReadTicks
    sub ax, si
IFNDEF EXPERIMENTAL_BAUD
    cmp ax, 08000h
    jae UART_PutChar_Loop
ENDIF
    cmp ax, UART_TIMEOUT_TICKS
    jb UART_PutChar_Loop
    jmp short UART_PutChar_Fail
UART_PutChar_Write:
    mov dx, word ptr cs:[UART_BASE]
    mov al, bl
    out dx, al
    pop ax
    clc
    jmp short UART_PutChar_Done
UART_PutChar_Fail:
    pop ax
    stc
UART_PutChar_Done:
    pop es
    pop ds
    pop bp
    pop di
    pop si
    pop dx
    pop cx
    pop bx
    ret
UART_PutChar ENDP

UART_FlushInput PROC NEAR
    push ax
    push dx
UART_FlushInput_Loop:
    call UART_PollCancel
    jc UART_FlushInput_Fail
    mov dx, word ptr cs:[UART_BASE]
    or dx, dx
    jz UART_FlushInput_Ok
    add dx, 5
    in al, dx
    test al, 001h
    jz UART_FlushInput_Ok
    mov dx, word ptr cs:[UART_BASE]
    in al, dx
    jmp short UART_FlushInput_Loop
UART_FlushInput_Ok:
    clc
    jmp short UART_FlushInput_Done
UART_FlushInput_Fail:
    stc
UART_FlushInput_Done:
    pop dx
    pop ax
    ret
UART_FlushInput ENDP

UART_CtrlBreakHandler PROC FAR
    mov byte ptr cs:[UART_CANCEL], 1
    iret
UART_CtrlBreakHandler ENDP

UART_BASE       DW 0
UART_OLD_LCR    DB 0
UART_OLD_MCR    DB 0
UART_OLD_DLL    DB 0
UART_OLD_DLM    DB 0
UART_CANCEL     DB 0
UART_BREAK_HOOK DB 0
UART_OLD_BREAK_OFF DW 0
UART_OLD_BREAK_SEG DW 0
UART_BIOS_RATE DB 0
UART_PORT_NUM   DB 0

include uart_guard.inc
