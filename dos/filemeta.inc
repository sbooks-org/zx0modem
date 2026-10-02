; Version-1 transfer metadata uses DOS packed local time/date and readonly only.
; CX=time, DX=date. CF=1 for impossible civil dates or reserved time values.
; All input registers preserved. DOS date's seven year bits cover 1980..2107.
MD_FAIL_IF MACRO condition
    LOCAL valid
    condition valid
    jmp MD_invalid
valid:
ENDM

MD_Validate PROC NEAR
    push ax
    push bx
    push cx
    push dx
    mov ax,cx
    and ax,001Fh
    cmp ax,29
    MD_FAIL_IF jbe
    mov ax,cx
    shr ax,5
    and ax,003Fh
    cmp ax,59
    MD_FAIL_IF jbe
    mov ax,cx
    shr ax,11
    cmp ax,23
    MD_FAIL_IF jbe
    mov bx,dx
    shr bx,5
    and bx,000Fh
    cmp bx,1
    MD_FAIL_IF jae
    cmp bx,12
    MD_FAIL_IF jbe
    mov al,[MD_month_days+bx-1]
    cmp bx,2
    jne short MD_day
    mov bx,dx
    shr bx,9
    ; 1980 is divisible by four; 2100 (offset 120) is not a leap year.
    test bl,3
    jnz short MD_day
    cmp bx,120
    je short MD_day
    inc al
MD_day:
    and dx,001Fh
    or dx,dx
    MD_FAIL_IF jnz
    xor ah,ah
    cmp dx,ax
    MD_FAIL_IF jbe
    pop dx
    pop cx
    pop bx
    pop ax
    clc
    ret
MD_invalid:
    pop dx
    pop cx
    pop bx
    pop ax
    stc
    ret
MD_Validate ENDP

MD_month_days DB 31,28,31,30,31,30,31,31,30,31,30,31
