################################################################################

.data

str1:
  .asciiz "Enter the number of integers to insert into the array: "
str2:
  .asciiz "Enter a 32-bit integer to insert into the array (in decimal): "

newline:
  .asciiz "\n"
space:
  .asciiz " "

n:
  .align 2
  .space 4
num_ints:
  .align 2
  .word 0 # <-- initially, there are 0 integer in the array
array:
  .align 2
  .space 4096 # <-- capable of storing up to 1024 32-bit integers

################################################################################

.text

# $a0 = n, $a1 = &array[0]
print_array:
  move $t0, $a0
  li $t1, 0
  move $t2, $a1
print_array_loop:
  beq $t1, $t0, print_array_return
  li $v0, 1
  lw $a0, ($t2)
  syscall
  li $v0, 4
  la $a0, space
  syscall
  addi $t1, $t1, 1
  addi $t2, $t2, 4
  j print_array_loop
print_array_return:
  li $v0, 4
  la $a0, newline
  syscall
  jr $ra

# $a0 = new_int, $a1 = num_ints, $a2 = &array[0]
sorted_array_insert:
################################################################################

  li $t0, 0 #t0 : i=0

find_pos :
  bge $t0, $a1, insert_at_end #if i >= num_ints, insert at end

  #find this element's address by t1
  sll $t1, $t0, 2 #t1 =i*4
  add $t1, $t1, $a2 #t1 = &array[0]+i*4
  
  #load array[i] by t2
  lw $t2, 0($t1)

  #compare
  bge $a0, $t2, continue #if new_int >= arr[i] check next value

  #if we found the index to insert
  j shift_elements

continue:
  addi $t0, $t0, 1 #i++
  j find_pos

insert_at_end:
  sll $t1, $a1, 2 #t1 : num_ints*4
  add $t1, $t1, $a2 #t1 = address of the end of the array
  sw $a0, 0($t1)
  j done

shift_elements:
  move $t3, $a1 #t3 = array size(j)
shift_loop:
  beq $t3, $t0, insert_at_position #j==insert pos -> escape
  #get j-1 element value
  sub $t4, $t3, 1 #t4=j-1
  sll $t4, $t4, 2 #t4=(j-1)*4
  add $t4, $t4, $a2 #t4=&array[0]+(j-1)*4
  lw $t5, 0($t4) #t5 = arr[j-1]
  
  # arr[j] = arr[j-1]
  sll $t6, $t3, 2 #t6=j*4
  add $t6, $t6, $a2 #$t6=&array[0]+(j-1)*4
  sw $t5, 0($t6) #a[j]=a[j-1]

  sub $t3, $t3, 1
  j shift_loop

insert_at_position:
  sll $t1, $t0, 2
  add $t1, $t1, $a2
  sw $a0, 0($t1)

done:

################################################################################
  jr $ra

.globl main
main:
  # $s0 = $ra
  move $s0, $ra

  # print_string(str1); read_int (--> $v0); *n = $v0
  li $v0, 4
  la $a0, str1
  syscall
  li $v0, 5
  syscall
  sw $v0, n

  move $s1, $v0

gen_ints_0:

  beq $s1, $zero, gen_ints_1

  # print_string(str2); read_int (--> $v0) --> $s2
  li $v0, 4
  la $a0, str2
  syscall
  li $v0, 5
  syscall
  move $s2, $v0

  # sorted_array_insert($s2, *num_ints, &array[0]); (*num_ints)++;
  move $a0, $s2
  lw $a1, num_ints
  la $a2, array
  jal sorted_array_insert
  lw $s2, num_ints
  addi $s2, $s2, 1
  sw $s2, num_ints

  # print_array(*num_ints, &array[0])
  lw $a0, num_ints
  la $a1, array
  jal print_array

  addi $s1, $s1, -1

  beq $zero, $zero, gen_ints_0

gen_ints_1:

  # $ra = $s0
  move $ra, $s0
 
  # return
  jr $ra

