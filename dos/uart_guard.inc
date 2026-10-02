; Temporary ownership of a polled UART on a PC/AT-compatible BIOS.
; Serial-console BIOSes can drain RX in IRQ0 even with UART interrupts off.
; Replace that background poll while retaining the BIOS clock and INT 1Ch.
; BIOS-specific background services (including USB keyboard polling) resume
; on release. Do not use alongside a TSR that also owns this UART or IRQ0.
; Acquire: DX = UART base, DLAB clear. All registers and flags preserved.
; Release: restores IRQ0 and IER, retaining the caller's baud/LCR/MCR.

UART_GuardAcquire PROC NEAR
    pushf
    cli
    push ax
    push dx
    push es
    cmp byte ptr cs:[UART_GUARD_ACTIVE],0
    jne short UART_GuardAcquire_Done
    mov word ptr cs:[UART_GUARD_BASE],dx
    inc dx
    in al,dx
    mov byte ptr cs:[UART_GUARD_IER],al
    xor ax,ax
    out dx,al
    mov es,ax
    mov ax,word ptr es:[20h]
    mov word ptr cs:[UART_GUARD_OLD_OFF],ax
    mov ax,word ptr es:[22h]
    mov word ptr cs:[UART_GUARD_OLD_SEG],ax
    mov word ptr es:[20h],OFFSET UART_GuardTick
    mov ax,cs
    mov word ptr es:[22h],ax
    mov byte ptr cs:[UART_GUARD_ACTIVE],1
UART_GuardAcquire_Done:
    pop es
    pop dx
    pop ax
    popf
    ret
UART_GuardAcquire ENDP

UART_GuardRelease PROC NEAR
    pushf
    cli
    push ax
    push dx
    push es
    cmp byte ptr cs:[UART_GUARD_ACTIVE],0
    je short UART_GuardRelease_Done
    xor ax,ax
    mov es,ax
    mov ax,word ptr cs:[UART_GUARD_OLD_OFF]
    mov word ptr es:[20h],ax
    mov ax,word ptr cs:[UART_GUARD_OLD_SEG]
    mov word ptr es:[22h],ax
    mov dx,word ptr cs:[UART_GUARD_BASE]
    add dx,3
    in al,dx
    mov ah,al
    and al,7Fh
    out dx,al
    sub dx,2
    mov al,byte ptr cs:[UART_GUARD_IER]
    out dx,al
    add dx,2
    mov al,ah
    out dx,al
    mov byte ptr cs:[UART_GUARD_ACTIVE],0
UART_GuardRelease_Done:
    pop es
    pop dx
    pop ax
    popf
    ret
UART_GuardRelease ENDP

UART_GuardTick PROC FAR
    pusha
    push ds
    push es
    mov ax,0040h
    mov ds,ax
    add word ptr [006Ch],1
    adc word ptr [006Eh],0
    cmp word ptr [006Eh],0018h
    jb short UART_GuardTick_Chain
    ja short UART_GuardTick_Midnight
    cmp word ptr [006Ch],00B0h
    jb short UART_GuardTick_Chain
UART_GuardTick_Midnight:
    mov word ptr [006Ch],0
    mov word ptr [006Eh],0
    inc byte ptr [0070h]
UART_GuardTick_Chain:
    ; Do not chain the original IRQ0 handler: that is the competing RX reader.
    ; Leave the floppy motor countdown for that handler after release. Disk
    ; IRQs remain enabled and BIOS disk timeouts still see advancing ticks.
    int 1Ch
    mov al,20h
    out 20h,al
    pop es
    pop ds
    popa
    iret
UART_GuardTick ENDP

UART_GUARD_ACTIVE DB 0
UART_GUARD_BASE DW 0
UART_GUARD_IER DB 0
UART_GUARD_OLD_OFF DW 0
UART_GUARD_OLD_SEG DW 0
