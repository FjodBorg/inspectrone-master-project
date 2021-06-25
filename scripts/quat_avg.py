
import numpy as np

#Q = np.array([[-1,0,0,1],[0,0,1,1]])
Q = np.array([[ 0.258819, 0, 0, 0.9659258 ], [ 0.8660254, 0, 0, 0.5 ]])
w = np.array([0, 1])

# Number of quaternions to average
M = Q.shape[0]
A = np.zeros(shape=(4, 4))
weightSum = 0
print (Q)
for i in range(0, M):
	q = Q[i, :]
	A = w[i] * np.outer(q, q) + A
	weightSum += w[i]

	# print(w[i])
	#print(w[i] * np.outer(q, q))
	#q = q.reshape(1,4)
	#print(w[i] * q.T @ q) #correct

	#print()

# scale
print(A)
A = (1.0 / weightSum) * A

#test = np.diag(w) @ Q
#print(test)
#print(np.transpose(test) @ test)
#print(np.transpose(Q)  @  Q)
#print(np.conjugate(A))
# print(np.matmul(np.diag(w), Q))

# compute eigenvalues and -vectors
eigenValues, eigenVectors = np.linalg.eig(A)

# Sort by largest eigenvalue
print(np.real(eigenVectors))
eigenVectors = eigenVectors[:, eigenValues.argsort()[::-1]]
print(np.real(eigenVectors))
print(eigenValues)


# return the real part of the largest eigenvector (has only real part)
print(np.real(eigenVectors[:, 0]))


'''
Averaging Quaternions.

Arguments:
Q(ndarray): an Mx4 ndarray of quaternions.
weights(list): an M elements list, a weight for each quaternion.
'''

# Form the symmetric accumulator matrix
A = np.zeros((4, 4))
M = Q.shape[0]
wSum = 0

for i in range(M):
	q = Q[i, :]
	w_i = w[i]
	A += w_i * (np.outer(q, q)) # rank 1 update
	wSum += w_i

# scale
A /= wSum

print(np.linalg.eigh(A)[1][:, -1])
