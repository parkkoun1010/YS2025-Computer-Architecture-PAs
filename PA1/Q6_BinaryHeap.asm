################################################################################

.data

str1:
  .asciiz "Enter the number of integers to push into the min heap: "
str2:
  .asciiz "Enter a 32-bit integer to push into the min heap (in decimal): "
str3:
  .asciiz "INFO: Popping a 32-bit integer from the min heap:\n"
str4:
  .asciiz "INFO: min_heap_pop returned:\n"
str5:
  .asciiz "INFO:   $v0 = "
newline:
  .asciiz "\n"
space:
  .asciiz " "

min_heap:
  .align 2
  .space 4096

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

# $a0: the current number of elements in the binary min heap
# $a1: the starting memory address of the binary min heap
# $a2: the new 32-bit signed integer to push to the binary min heap
min_heap_push:
################################################################################

  sll $t0, $a0, 2       # $t0 = n * 4
  add $t0, $a1, $t0     # $t0 = heap[n]'s address
  sw $a2, 0($t0)        # heap[n] = new val
    
  #bubble up (i=n)
  move $t1, $a0         # $t1 = cur index (i)

bubble_up:
  beq $t1, $zero, end_push
  
  #t2 = parent index
  sub $t2, $t1, 1 #t2=i-1
  srl $t2, $t2, 1 #t2=(i-1)/2
  
  #find cur node address->t3
  sll $t3, $t1, 2 #t3=i*4
  add $t3, $a1, $t3 #t3=&heap[i]
  #t4 = heap[i]
  lw $t4, 0($t3)

  #find parent node address
  sll $t5, $t2, 2 #t5 = parent *4
  add $t5, $a1, $t5 #t5 = &heap[parent]
  lw $t6, 0($t5) #t6 = heap[parent]

  #value compare
  bge $t4, $t6, end_push #if heap[i]>=heap[parent] end
  
  #value exchange
  sw $t6, 0($t3) #heap[i]=heap[parent]
  sw $t4, 0($t5) #heap[parent]=heap[i]
  move $t1, $t2 #i=parent
  j bubble_up

end_push:
################################################################################
  jr $ra

# $a0: the current number of elements in the binary min heap
# $a1: the starting memory address of the binary min heap
# $v0: the popped/deleted 32-bit signed integer
min_heap_pop:
################################################################################
  # save heap[0] in v0
  lw $v0, 0($a1)

  # heap[n-1] to root
  sub $t0, $a0, 1 #t0=n-1
  sll $t1, $t0, 2 #t1 = (n-1)*4
  add $t1, $a1, $t1 #t1 = &heap[n-1]
  lw $t2, 0($t1) #t2 = heap[n-1]
  sw $t2, 0($a1) #heap[0] = heap[n-1]
  
  #bubble down
  li $t3,0 #let t3=i=0
  sub $a0, $a0, 1
  
bubble_down:
  #left =2*i+1
  sll $t4, $t3, 1 #t4=2i
  addi $t4, $t4, 1#t4=2i+1
  
  #end condition left>=n
  bge $t4, $a0,  end_pop
 
  #smallest = left(initialize)
  move $t5, $t4 #t5 = smallest
  
  #right=left+1(t6)
  addi $t6, $t4, 1
  
  #right<n
  blt $t6, $a0, check_right
  #compare smallest and curnode
  j compare

check_right:
  #compare heap[right] and heap[smallest]
  sll $t7, $t6, 2 #t7=right*4
  add $t7, $a1, $t7 #t7=&heap[right]
  lw $t8, 0($t7) #t8=heap[right]

  sll $t9, $t5, 2 #t9=smallest*4
  add $t9, $a1, $t9 #t9=&heap[smallest]
  lw $t9, 0($t9) #t9=heap[smallest]
  
  #if heap[right]=<heap[smallest]
  bge $t9, $t8, update_smallest

compare:
  #get heap[smallest]
  sll $t7, $t5, 2
  add $t7, $a1, $t7
  lw $t8, 0($t7)
  
  #get heap[i]
  sll $t9, $t3, 2
  add $t9, $a1, $t9
  lw $t0, 0($t9)

  #heap[smallest]>=heap[i] end
  bge $t8, $t0, end_pop  

  #exchange
  sw $t8, 0($t9)
  sw $t0, 0($t7)
  move $t3, $t5 #i=smallest
  j bubble_down

update_smallest:
  move $t5, $t6
  j compare

end_pop:

################################################################################
  jr $ra

.globl main
main:

  # stack << $ra
  addi $sp, $sp, -4
  sw $ra, 0($sp)

  # print_str(str1); $s0 = read_int()
  li $v0, 4
  la $a0, str1
  syscall
  li $v0, 5
  syscall
  move $s0, $v0

  # push $s0 integers into the binary min heap
  li $s1, 0
main_0:
  # print_str(str2)
  li $v0, 4
  la $a0, str2
  syscall
  # $s2 = read_int()
  li $v0, 5
  syscall
  move $s2, $v0
  # min_heap_push($s1, min_heap, $s2)
  move $a0, $s1
  la $a1, min_heap
  move $a2, $s2
  jal min_heap_push
  # $s1 = $s1 + 1
  addi $s1, $s1, 1
  # print_array($s1, min_heap)
  move $a0, $s1
  la $a1, min_heap
  jal print_array
  # if ($s1 < $s0) then goto main_0
  slt $t0, $s1, $s0
  bne $t0, $zero, main_0

main_1:

  # pop $s0 integers from the binary min heap
  move $s2, $s1
  li $s1, 0
main_2:
  # print_str(str3)
  li $v0, 4
  la $a0, str3
  syscall
  # min_heap_pop($s2, min_heap)
  move $a0, $s2
  la $a1, min_heap
  jal min_heap_pop
  # $s3 = $v0
  move $s3, $v0
  # print_str(str4)
  li $v0, 4
  la $a0, str4
  syscall
  # print_str(str5)
  li $v0, 4
  la $a0, str5
  syscall
  # print_int($s3)
  li $v0, 1
  move $a0, $s3
  syscall
  # print_str(newline)
  li $v0, 4
  la $a0, newline
  syscall
  
  # $s1 = $s1 + 1
  # $s2 = $s2 - 1
  addi $s1, $s1, 1
  addi $s2, $s2, -1

  # print_array($s2, min_heap)
  move $a0, $s2
  la $a1, min_heap
  jal print_array

  # if ($s1 < $s0) then goto main_2
  slt $t0, $s1, $s0
  bne $t0, $zero, main_2

main_3:

  # stack >> $ra
  lw $ra, 0($sp)
  addi $sp, $sp, 4

  jr $ra

