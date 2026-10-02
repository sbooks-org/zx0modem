; ZX0 v2 forward bitstream decoder and bounded block framing.
.286
ZX0_MAX_BLOCK EQU 4096
ZX_JC MACRO target
    LOCAL skip
    jnc     short skip
    jmp     target
skip:
ENDM
ZX_JNE MACRO target
    LOCAL skip
    je      short skip
    jmp     target
skip:
ENDM
ZX_JA MACRO target
    LOCAL skip
    jbe     short skip
    jmp     target
skip:
ENDM
ZX_JE MACRO target
    LOCAL skip
    jne     short skip
    jmp     target
skip:
ENDM
ZX_JZ MACRO target
    LOCAL skip
    jnz     short skip
    jmp     target
skip:
ENDM

; Decode a single compressed block.
; Input: DS:SI encoded bytes, CX encoded length, ES:DI output, BP expected bytes.
; Output: CF clear on exact size and no trailing bytes; sets DI past output.
ZX0_Decode PROC NEAR
    push    ax
    push    bx
    push    cx
    push    dx
    push    si
    push    bp
    mov     [zx_input],si
    mov     [zx_input_len],cx
    mov     [zx_output_start],di
    mov     [zx_expected],bp
    mov     word ptr [zx_input_at],0
    mov     word ptr [zx_bit_value],08000h
    mov     word ptr [zx_offset],1
    call    ZX0_Gamma0
    ZX_JC ZX0_D_bad
    mov     [zx_literal],ax
ZX0_D_outer:
    mov     ax,di
    sub     ax,[zx_output_start]
    mov     dx,[zx_expected]
    sub     dx,ax
    cmp     [zx_literal],dx
    ZX_JA ZX0_D_bad
    mov     cx,[zx_literal]
ZX0_D_literals:
    jcxz    short ZX0_D_after_literals
    call    ZX0_GetByte
    ZX_JC ZX0_D_bad
    stosb
    loop    ZX0_D_literals
ZX0_D_after_literals:
    call    ZX0_GetBit
    ZX_JC ZX0_D_bad
    or      al,al
    ZX_JZ ZX0_D_last_offset
ZX0_D_new_offset:
    mov     byte ptr [zx_inverted],1
    call    ZX0_Gamma
    ZX_JC ZX0_D_bad
    cmp     ax,256
    ZX_JE ZX0_D_end_marker
    cmp     ax,32
    ZX_JA ZX0_D_bad
    shl     ax,1
    shl     ax,1
    shl     ax,1
    shl     ax,1
    shl     ax,1
    shl     ax,1
    shl     ax,1
    mov     [zx_work],ax
    call    ZX0_GetByte
    ZX_JC ZX0_D_bad
    mov     [zx_low_byte],al
    shr     al,1
    xor     ah,ah
    mov     dx,[zx_work]
    sub     dx,ax
    or      dx,dx
    ZX_JZ ZX0_D_bad
    mov     [zx_offset],dx
    mov     ax,[zx_bit_value]
    shr     ax,1
    test    byte ptr [zx_low_byte],1
    jz      short ZX0_D_backtracked
    or      ax,8000h
ZX0_D_backtracked:
    mov     [zx_bit_value],ax
    call    ZX0_Gamma0
    ZX_JC ZX0_D_bad
    inc     ax
    ZX_JC ZX0_D_bad
    mov     [zx_match],ax
    call    ZX0_CopyMatch
    ZX_JC ZX0_D_bad
    call    ZX0_GetBit
    ZX_JC ZX0_D_bad
    or      al,al
    jz      short ZX0_D_new_offset_done
    jmp     ZX0_D_new_offset
ZX0_D_new_offset_done:
    call    ZX0_Gamma0
    ZX_JC ZX0_D_bad
    mov     [zx_literal],ax
    jmp     ZX0_D_outer
ZX0_D_last_offset:
    call    ZX0_Gamma0
    ZX_JC ZX0_D_bad
    mov     [zx_match],ax
    call    ZX0_CopyMatch
    ZX_JC ZX0_D_bad
    call    ZX0_GetBit
    ZX_JC ZX0_D_bad
    or      al,al
    jnz     short ZX0_D_offset_loop
    call    ZX0_Gamma0
    ZX_JC ZX0_D_bad
    mov     [zx_literal],ax
    jmp     ZX0_D_outer
ZX0_D_offset_loop:
    mov     byte ptr [zx_inverted],1
    call    ZX0_Gamma
    ZX_JC ZX0_D_bad
    cmp     ax,256
    je      short ZX0_D_end_marker
    cmp     ax,32
    ZX_JA ZX0_D_bad
    shl     ax,1
    shl     ax,1
    shl     ax,1
    shl     ax,1
    shl     ax,1
    shl     ax,1
    shl     ax,1
    mov     [zx_work],ax
    call    ZX0_GetByte
    ZX_JC ZX0_D_bad
    mov     [zx_low_byte],al
    shr     al,1
    xor     ah,ah
    mov     dx,[zx_work]
    sub     dx,ax
    or      dx,dx
    ZX_JZ ZX0_D_bad
    mov     [zx_offset],dx
    mov     ax,[zx_bit_value]
    shr     ax,1
    test    byte ptr [zx_low_byte],1
    jz      short ZX0_D_backtracked2
    or      ax,8000h
ZX0_D_backtracked2:
    mov     [zx_bit_value],ax
    call    ZX0_Gamma0
    ZX_JC ZX0_D_bad
    inc     ax
    ZX_JC ZX0_D_bad
    mov     [zx_match],ax
    call    ZX0_CopyMatch
    ZX_JC ZX0_D_bad
    call    ZX0_GetBit
    ZX_JC ZX0_D_bad
    or      al,al
    jnz     short ZX0_D_offset_loop
    call    ZX0_Gamma0
    ZX_JC ZX0_D_bad
    mov     [zx_literal],ax
    jmp     ZX0_D_outer
ZX0_D_end_marker:
    mov     ax,di
    sub     ax,[zx_output_start]
    cmp     ax,[zx_expected]
    ZX_JNE ZX0_D_bad
    mov     ax,[zx_input_at]
    cmp     ax,[zx_input_len]
    ZX_JNE ZX0_D_bad
    pop     bp
    pop     si
    pop     dx
    pop     cx
    pop     bx
    pop     ax
    clc
    ret
ZX0_D_bad:
    pop     bp
    pop     si
    pop     dx
    pop     cx
    pop     bx
    pop     ax
    stc
    ret
ZX0_Decode ENDP

; Decode Elias gamma integer. zx_inverted = 0/1.
ZX0_Gamma0 PROC NEAR
    mov     byte ptr [zx_inverted],0
    jmp     short ZX0_Gamma
ZX0_Gamma0 ENDP
ZX0_Gamma PROC NEAR
    mov     ax,1
ZX0_Gamma_loop:
    ; GetBit returns AL and destroys AX: keep the gamma accumulator on
    ; the stack until the continuation bit has been inspected.
    push    ax
    call    ZX0_GetBit
    jc      short ZX0_Gamma_popbad
    or      al,al
    pop     ax
    jnz     short ZX0_Gamma_ok
    cmp     ax,8000h
    jae     short ZX0_Gamma_bad
    shl     ax,1
    push    ax
    call    ZX0_GetBit
    jc      short ZX0_Gamma_popbad
    xor     al,[zx_inverted]
    xor     ah,ah
    pop     dx
    or      ax,dx
    cmp     ax,4097
    ja      short ZX0_Gamma_bad
    jmp     short ZX0_Gamma_loop
ZX0_Gamma_popbad:
    pop     ax
    stc
    ret
ZX0_Gamma_ok:
    clc
    ret
ZX0_Gamma_bad:
    stc
    ret
ZX0_Gamma ENDP

; Forward ZX0 bit reader with interleaved raw bytes.
ZX0_GetBit PROC NEAR
    mov     ax,[zx_bit_value]
    cmp     ax,08000h
    jne     short ZX0_GB_test
    call    ZX0_GetByte
    jc      short ZX0_GB_bad
    mov     ah,al
    mov     al,080h
    mov     [zx_bit_value],ax
ZX0_GB_test:
    mov     ax,[zx_bit_value]
    test    ax,08000h
    jz      short ZX0_GB_zero
    mov     al,1
    shl     word ptr [zx_bit_value],1
    clc
    ret
ZX0_GB_zero:
    xor     al,al
    shl     word ptr [zx_bit_value],1
    clc
    ret
ZX0_GB_bad:
    stc
    ret
ZX0_GetBit ENDP

ZX0_GetByte PROC NEAR
    mov     ax,[zx_input_at]
    cmp     ax,[zx_input_len]
    jae     short ZX0_GetByte_bad
    mov     bx,[zx_input]
    add     bx,ax
    mov     al,[bx]
    inc     word ptr [zx_input_at]
    clc
    ret
ZX0_GetByte_bad:
    stc
    ret
ZX0_GetByte ENDP

ZX0_CopyMatch PROC NEAR
    mov     ax,di
    sub     ax,[zx_output_start]
    mov     dx,[zx_expected]
    sub     dx,ax
    cmp     [zx_match],dx
    ja      short ZX0_CM_bad
    cmp     ax,[zx_offset]
    jb      short ZX0_CM_bad
    mov     bx,di
    sub     bx,[zx_offset]
    mov     cx,[zx_match]
ZX0_CM_loop:
    jcxz    short ZX0_CM_ok
    mov     al,es:[bx]
    mov     es:[di],al
    inc     bx
    inc     di
    loop    ZX0_CM_loop
ZX0_CM_ok:
    clc
    ret
ZX0_CM_bad:
    stc
    ret
ZX0_CopyMatch ENDP


zx_input DW 0
zx_input_len DW 0
zx_input_at DW 0
zx_output_start DW 0
zx_expected DW 0
zx_offset DW 1
zx_literal DW 0
zx_match DW 0
zx_work DW 0
zx_bit_value DW 08000h
zx_low_byte DB 0
zx_inverted DB 0
