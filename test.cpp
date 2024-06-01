#include <gtest/gtest.h>
/*
TEST(SimpleTest, Addition) {
    ASSERT_EQ(2 + 2, 4);
}
*/

long mul(int n)
{
int i, m=1;
if (n==1)
	return m;
else
	for(int i=1; i<=n; i++)
		m=m*i;
	return m;
}

//factorial
int factorial(int n)
{
if (n<=1)
	return 1;
else
	return n*factorial(n-1);
}

//fabonaci
int fab(int n)
{
if (n<=1)
	return n;
else
	return fab(n-1) + fab(n-2);
}

TEST(FabTest, fabnaci)
{
    ASSERT_EQ(0, fab(0));
}

TEST(FactorialTest, FactorialOfZeroShouldBeOne)
{
    ASSERT_EQ(mul(1), factorial(0));
}

TEST(FactorialTest, FactorialOfPositiveNos)
{
    ASSERT_EQ(mul(1), factorial(1));
    ASSERT_EQ(mul(4), factorial(4));
    ASSERT_EQ(mul(5), factorial(5));
}


int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

