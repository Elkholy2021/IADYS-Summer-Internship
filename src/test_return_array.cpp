#include <iostream>
using namespace std;

struct demo
{
	//array declared inside structure
	int arr[2][10];
};

struct demo func(int n) //return type is struct demo
{
	struct demo demo_mem; //demo structure member declared
	for(int i=0;i<n;i++)
	{
		//array initialisation
		demo_mem.arr[0][i] = i;
        demo_mem.arr[1][i] = 1;
	}
	return demo_mem; //address of structure member returned
}

int main() 
{
	struct demo a;
	int n=5; //number of elements
	
	a=func(n); //address of arr
	
	cout<<"The Array is : ";
	for(int i=0;i<n;i++)
	{
		cout<<a.arr[0][i]<<"\t";
        cout<<a.arr[1][i]<<"\t";
	}
	
	return 0;
}
