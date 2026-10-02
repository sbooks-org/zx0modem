; ZRECVX-only training. No DOS/BIOS console I/O while protocol owns the UART.
; Baseline commands are at 9600. Each probe has a ten-second overall deadline
; in addition to one-second byte/TX deadlines; command idle is five seconds.
BT_IDLE_TICKS EQU 91
BT_PROBE_TICKS EQU 182
BT_LOCK_TICKS EQU 91

; Polling cancellation is sticky here because the keyboard routine consumes it.
BT_Cancel PROC NEAR
    cmp byte ptr [BT_cancelled],0
    jne short BT_Cancel_bad
    call UART_PollCancel
    jnc short BT_Cancel_ok
    mov byte ptr [BT_cancelled],1
BT_Cancel_bad:
    stc
    ret
BT_Cancel_ok:
    clc
    ret
BT_Cancel ENDP

; CF if the current overall phase expired. Unsigned elapsed also terminates
; at BIOS midnight rollover instead of waiting for a missing tick value.
BT_CheckLimit PROC NEAR
    push ax
    call BT_Cancel
    jc short BT_CheckLimit_bad
    call UART_ReadTicks
    sub ax,[BT_phase_start]
    cmp ax,[BT_phase_limit]
    jae short BT_CheckLimit_bad
    pop ax
    clc
    ret
BT_CheckLimit_bad:
    pop ax
    stc
    ret
BT_CheckLimit ENDP

BT_StartIdle PROC NEAR
    push ax
    call UART_ReadTicks
    mov [BT_phase_start],ax
    mov word ptr [BT_phase_limit],BT_IDLE_TICKS
    pop ax
    ret
BT_StartIdle ENDP

; AL byte, CF failure. LSR overrun/parity/framing/break errors fail a probe,
; even when DR is clear. Preserve loop/pattern registers across every byte.
BT_GetByte PROC NEAR
    push bx
    push dx
    push si
    call UART_ReadTicks
    mov si,ax
BT_GetByte_loop:
    call BT_CheckLimit
    jc short BT_GetByte_bad
    mov dx,[UART_BASE]
    add dx,5
    in al,dx
    test al,01Eh
    jnz short BT_GetByte_bad
    test al,1
    jnz short BT_GetByte_ready
    call UART_ReadTicks
    sub ax,si
    cmp ax,18
    jae short BT_GetByte_bad
    jmp short BT_GetByte_loop
BT_GetByte_ready:
    sub dx,5
    in al,dx
    pop si
    pop dx
    pop bx
    clc
    ret
BT_GetByte_bad:
    ; The shared decoder uses CF for both terminators and UART errors.
    ; Latch failures so an error can never masquerade as a valid terminator.
    mov byte ptr [BT_probe_fault],1
    pop si
    pop dx
    pop bx
    stc
    ret
BT_GetByte ENDP

; CX ticks, bounded and cancel-aware; preserve all registers.
BT_WaitTicks PROC NEAR
    push ax
    push si
    call UART_ReadTicks
    mov si,ax
BT_WaitTicks_loop:
    call BT_Cancel
    jc short BT_WaitTicks_bad
    call UART_ReadTicks
    sub ax,si
    cmp ax,cx
    jb short BT_WaitTicks_loop
    pop si
    pop ax
    clc
    ret
BT_WaitTicks_bad:
    pop si
    pop ax
    stc
    ret
BT_WaitTicks ENDP

; THRE alone is insufficient before changing divisor: wait for TEMT too.
BT_Drain PROC NEAR
    push ax
    push dx
    push si
    call UART_ReadTicks
    mov si,ax
BT_Drain_loop:
    call BT_Cancel
    jc short BT_Drain_bad
    mov dx,[UART_BASE]
    add dx,5
    in al,dx
    test al,040h
    jnz short BT_Drain_ok
    call UART_ReadTicks
    sub ax,si
    cmp ax,18
    jb short BT_Drain_loop
BT_Drain_bad:
    pop si
    pop dx
    pop ax
    stc
    ret
BT_Drain_ok:
    pop si
    pop dx
    pop ax
    clc
    ret
BT_Drain ENDP

; Bound flushing even if the peer continuously transmits stale probe bytes.
BT_Flush PROC NEAR
    push ax
    push dx
    push si
    call UART_ReadTicks
    mov si,ax
BT_Flush_loop:
    call BT_Cancel
    jc short BT_Flush_bad
    mov dx,[UART_BASE]
    add dx,5
    in al,dx
    test al,1
    jz short BT_Flush_ok
    sub dx,5
    in al,dx
    call UART_ReadTicks
    sub ax,si
    cmp ax,2
    jb short BT_Flush_loop
BT_Flush_ok:
    pop si
    pop dx
    pop ax
    clc
    ret
BT_Flush_bad:
    pop si
    pop dx
    pop ax
    stc
    ret
BT_Flush ENDP

; Restore baseline before checking cancellation, so every error is safe.
BT_Baseline PROC NEAR
    push ax
    push dx
    mov byte ptr [BT_locked],0
    mov byte ptr [BT_header_wait],0
    mov byte ptr [BT_probe_active],0
    xor dx,dx
    mov ax,9600
    call UART_SetBaud
    jc short BT_Baseline_done
    call BT_Flush
BT_Baseline_done:
    pop dx
    pop ax
    ret
BT_Baseline ENDP

; Process exit must release IRQ/console ownership via UART_Restore, at 9600.
BT_RestoreConsole PROC NEAR
    push ax
    push dx
    cmp word ptr [UART_BASE],0
    je short BT_RestoreConsole_done
    xor dx,dx
    mov ax,9600
    call UART_SetBaud
    mov byte ptr [UART_OLD_DLL],12
    mov byte ptr [UART_OLD_DLM],0
    mov byte ptr [UART_OLD_LCR],3
BT_RestoreConsole_done:
    pop dx
    pop ax
    ret
BT_RestoreConsole ENDP

BT_SetCandidate PROC NEAR
    push ax
    push bx
    push dx
    xor bx,bx
    mov bl,[BT_index]
    shl bx,1
    xor dx,dx
    mov ax,[BT_rates+bx]
    cmp bl,8
    jne short BT_SetCandidate_set
    inc dx
BT_SetCandidate_set:
    call UART_SetBaud
    pop dx
    pop bx
    pop ax
    ret
BT_SetCandidate ENDP

; Called from the header scanner only in X. A lost lock acknowledgement
; cannot leave the receiver stuck at the selected rate on an endless stream.
BT_HeaderCheck PROC NEAR
    push ax
    cmp byte ptr [BT_header_wait],0
    je short BT_HeaderCheck_ok
    call BT_Cancel
    jc short BT_HeaderCheck_bad
    call UART_ReadTicks
    sub ax,[BT_header_start]
    cmp ax,BT_LOCK_TICKS
    jae short BT_HeaderCheck_bad
BT_HeaderCheck_ok:
    pop ax
    clc
    ret
BT_HeaderCheck_bad:
    pop ax
    stc
    ret
BT_HeaderCheck ENDP

BT_Train PROC NEAR
    mov byte ptr [BT_verified],0
    mov byte ptr [BT_verified+1],0
    mov byte ptr [BT_verified+2],0
    mov byte ptr [BT_verified+3],0
    mov byte ptr [BT_verified+4],0
    mov al,'B'
    call UART_PutChar
    F_JC BT_train_error
    mov al,'T'
    call UART_PutChar
    F_JC BT_train_error
    mov al,'1'
    call UART_PutChar
    F_JC BT_train_error
    mov al,10
    call UART_PutChar
    F_JC BT_train_error
    call BT_StartIdle
BT_command:
    call BT_GetByte
    jnc short BT_command_byte
    call BT_CheckLimit
    F_JC BT_train_exit
    ; A per-byte timeout alone is not the five-second command idle timeout.
    jmp BT_command
BT_command_byte:
    cmp al,'X'
    F_JE BT_train_exit
    cmp al,'P'
    je short BT_command_index
    cmp al,'S'
    je short BT_command_index
    cmp al,'L'
    jne short BT_command
BT_command_index:
    mov [BT_command_kind],al
    call BT_GetByte
    F_JC BT_train_exit
    cmp al,4
    ja short BT_command
    mov [BT_index],al
    xor bx,bx
    mov bl,al
    cmp byte ptr [BT_command_kind],'P'
    F_JE BT_probe
    cmp byte ptr [BT_command_kind],'L'
    F_JE BT_lock
    mov al,'B'
    call UART_PutChar
    F_JC BT_train_error
    mov al,[BT_index]
    call UART_PutChar
    F_JC BT_train_error
    mov al,[BT_verified+bx]
    call UART_PutChar
    F_JC BT_train_error
    call BT_StartIdle
    jmp BT_command

BT_probe:
    mov byte ptr [BT_verified+bx],0
    mov byte ptr [BT_probe_pass],0
    mov byte ptr [BT_probe_fault],0
    mov al,'A'
    call UART_PutChar
    F_JC BT_probe_end
    mov al,[BT_index]
    call UART_PutChar
    F_JC BT_probe_end
    call BT_Drain
    F_JC BT_probe_end
    mov cx,2
    call BT_WaitTicks
    F_JC BT_probe_end
    call BT_SetCandidate
    F_JC BT_probe_end
    call UART_ReadTicks
    mov [BT_phase_start],ax
    mov word ptr [BT_phase_limit],BT_PROBE_TICKS
    mov al,'R'
    call UART_PutChar
    F_JC BT_probe_end
    mov al,[BT_index]
    call UART_PutChar
    F_JC BT_probe_end
    mov byte ptr [BT_round],0
BT_probe_round:
    ; Exercise the actual transfer unescaping and CRC16 receive path.
    mov byte ptr [BT_probe_active],1
    mov di,OFFSET data_buf
    mov cx,512
    call ZM_RecvData16
    F_JC BT_probe_end
    cmp byte ptr [BT_probe_fault],0
    F_JNE BT_probe_end
    cmp cx,512
    F_JNE BT_probe_end
    cmp byte ptr [zm_endchar],ZCRCW
    F_JNE BT_probe_end
    mov al,[BT_index]
    mov bl,19
    mul bl
    mov dl,al
    mov al,[BT_round]
    mov bl,29
    mul bl
    add al,dl
    mov [BT_expected],al
    mov cx,512
    mov si,OFFSET data_buf
BT_probe_byte:
    lodsb
    cmp al,[BT_expected]
    F_JNE BT_probe_end
    add byte ptr [BT_expected],73
    loop BT_probe_byte
    mov al,'K'
    call UART_PutChar
    F_JC BT_probe_end
    mov al,[BT_index]
    call UART_PutChar
    F_JC BT_probe_end
    mov al,[BT_round]
    call UART_PutChar
    F_JC BT_probe_end
    inc byte ptr [BT_round]
    cmp byte ptr [BT_round],4
    F_JB BT_probe_round
    mov byte ptr [BT_probe_pass],1
BT_probe_end:
    mov byte ptr [BT_probe_active],0
    call BT_Drain
    jnc short BT_probe_drained
    mov byte ptr [BT_probe_pass],0
BT_probe_drained:
    mov cx,4
    call BT_WaitTicks
    jnc short BT_probe_guarded
    mov byte ptr [BT_probe_pass],0
BT_probe_guarded:
    call BT_Baseline
    F_JC BT_train_error
    xor bx,bx
    mov bl,[BT_index]
    mov al,[BT_probe_pass]
    mov [BT_verified+bx],al
    call BT_StartIdle
    jmp BT_command

BT_lock:
    cmp byte ptr [BT_verified+bx],1
    F_JNE BT_command
    mov al,'L'
    call UART_PutChar
    F_JC BT_train_error
    mov al,[BT_index]
    call UART_PutChar
    F_JC BT_train_error
    call BT_Drain
    F_JC BT_train_error
    mov cx,2
    call BT_WaitTicks
    F_JC BT_train_error
    call BT_SetCandidate
    F_JC BT_train_error
    mov cx,4
    call BT_WaitTicks
    F_JC BT_train_error
    mov byte ptr [BT_locked],1
    mov byte ptr [BT_header_wait],1
    call UART_ReadTicks
    mov [BT_header_start],ax
    clc
    ret
BT_train_exit:
    call BT_Baseline
    ret
BT_train_error:
    call BT_Baseline
    stc
    ret
BT_Train ENDP

BT_rates DW 9600,19200,38400,57600,49664
BT_verified DB 5 DUP(0)
BT_index DB 0
BT_round DB 0
BT_expected DB 0
BT_probe_pass DB 0
BT_command_kind DB 0
BT_locked DB 0
BT_header_wait DB 0
BT_header_start DW 0
BT_cancelled DB 0
BT_phase_start DW 0
BT_phase_limit DW BT_IDLE_TICKS
BT_can_train DB 1
BT_file_done DB 0
BT_probe_active DB 0
BT_probe_fault DB 0
