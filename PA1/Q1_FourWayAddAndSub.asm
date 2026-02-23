.data

newline:
  .asciiz "\n"

str0:
  .asciiz "Enter a 32-bit signed integer (in decimal): "

str1:
  .asciiz "INFO: fourWayAddAndSub returned:\n"
str2:
  .asciiz "INFO:   $v0 = "
str3:
  .asciiz "INFO:   $v1 = "

.text

# $a0 ~ $a3: 32-bit signed integers
fourWayAddAndSub:
################################################################################

  add $t0, $a0, $a1
  add $t1, $t0, $a2
  add $v0, $t1, $a3  
  
  sub $t2, $a0, $a1
  sub $t3, $t2, $a2
  sub $v1, $t3, $a3

################################################################################
  jr $ra

.globl main
main:

  # print_string str0; $t0 = read_int
  li $v0, 4
  la $a0, str0
  syscall
  li $v0, 5
  syscall
  move $t0, $v0

  # print_string str0; $t1 = read_int
  li $v0, 4
  la $a0, str0
  syscall
  li $v0, 5
  syscall
  move $t1, $v0

  # print_string str0; $t2 = read_int
  li $v0, 4
  la $a0, str0
  syscall
  li $v0, 5
  syscall
  move $t2, $v0

  # print_string str0; $t3 = read_int
  li $v0, 4
  la $a0, str0
  syscall
  li $v0, 5
  syscall
  move $t3, $v0

  # $s0 = $ra; $s1 = $a0; $s2 = $a1; $s3 = $a2; $s4 = $a3
  move $s0, $ra
  move $s1, $a0
  move $s2, $a1
  move $s3, $a2
  move $s4, $a3

  # call fourWayAddAndSub
  move $a0, $t0
  move $a1, $t1
  move $a2, $t2
  move $a3, $t3
  jal fourWayAddAndSub

  # $ra = $s0; $a0 = $s1; $a1 = $s2; $a2 = $s3; $a3 = $s4
  move $ra, $s0
  move $a0, $s1
  move $a1, $s2
  move $a2, $s3
  move $a3, $s4

  # $t0 = $v0; $t1 = $v1
  move $t0, $v0
  move $t1, $v1

  # print_string str1
  li $v0, 4
  la $a0, str1
  syscall

  # print_string str2; print_int $t0; print_string newline
  li $v0, 4
  la $a0, str2
  syscall
  li $v0, 1
  move $a0, $t0
  syscall
  li $v0, 4
  la $a0, newline
  syscall

  # print_string str3; print_int $t1; print_string newline
  li $v0, 4
  la $a0, str3
  syscall
  li $v0, 1
  move $a0, $t1
  syscall
  li $v0, 4
  la $a0, newline
  syscall

  # return 0
  li $v0, 0
  jr $ra

