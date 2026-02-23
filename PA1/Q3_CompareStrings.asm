################################################################################

.data

newline:
  .asciiz "\n"
str1:
  .asciiz "Enter the first string: "
str2:
  .asciiz "Enter the second string: "
str3:
  .asciiz "INFO: compareStrings returned:\n"
str4:
  .asciiz "INFO:   $v0 = "

inputStr1:
  .align 2
  .space 1024  # <-- pre-allocate 1024 bytes for the first input string
inputStr2:
  .align 2
  .space 1024  # <-- pre-allocate 1024 bytes for the second input string

################################################################################

.text

# compareStrings
#
# $a0: the starting memory address of string0
# $a1: the starting memory address of string1
#
# $v0 =  1 (if string0 > string1)
#        0 (if string0 = string1)
#       -1 (if string0 < string1)
compareStrings:
################################################################################

loop:
  #backup address a0->t2, a1->t3
  move $t2, $a0
  move $t3, $a1
  
  #str length count:t0,t1
  li $t0, 0
  li $t1, 0

str1_len:
  lb $t4, 0($t2)
  beq $t4, $zero, str2_len
  addi $t0, $t0, 1
  addi $t2, $t2, 1
  j str1_len

str2_len:
  lb $t4, 0($t3)
  beq $t4, $zero, compare_len
  addi $t1, $t1, 1
  addi $t3, $t3, 1
  j str2_len

compare_len:
  bne $t0, $t1, len_diff
  j char_compare

len_diff:
  slt $t2, $t0, $t1
  bne $t2, $zero, str2_larger
  li $v0, 1
  j end

str2_larger:
  li $v0, -1
  j end

char_compare:
  lb $t0, 0($a0)
  lb $t1, 0($a1)

  beq $t0, $zero, strings_equal
  bne $t0, $t1, char_diff
  
  addi $a0, $a0, 1
  addi $a1, $a1, 1
  j char_compare

char_diff:
  slt $t2, $t0, $t1
  bne $t2, $zero, str2_larger
  li $v0, 1
  j end

strings_equal:
  li $v0, 0

end:
################################################################################
  jr $ra

.globl main
main:

  # stack <-- $ra, $s0
  addi $sp, $sp, -8
  sw $ra, 0($sp)
  sw $s0, 4($sp)

  # print_string str1; read_string(inputStr1, $s0)
  li $v0, 4
  la $a0, str1
  syscall
  li $v0, 8
  la $a0, inputStr1
  li $a1, 1024
  syscall

  # print_string str2; read_string(inputStr2, $s1)
  li $v0, 4
  la $a0, str2
  syscall
  li $v0, 8
  la $a0, inputStr2
  li $a1, 1024
  syscall

  # compareStrings(inputStr1, inputStr2); $s2 = $v0
  la $a0, inputStr1
  la $a1, inputStr2
  jal compareStrings
  move $s2, $v0

  # print_string str3; print_string str4; print_int $s2; print_string newline
  li $v0, 4
  la $a0, str3
  syscall
  li $v0, 4
  la $a0, str4
  syscall
  li $v0, 1
  move $a0, $s2
  syscall
  li $v0, 4
  la $a0, newline
  syscall

  # $ra, $s0 <-- stack
  lw $ra, 0($sp)
  lw $s0, 4($sp)
  addi $sp, $sp, 8

  # return;
  jr $ra

################################################################################

