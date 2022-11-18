package org.firstinspires.ftc.teamcode.testing;

public class TestABS {
    public static void main(String []Args) {

        double testval=-0.05;

        double test;

        int repeats = 10;


        long before_mathabs = System.nanoTime();//System.currentTimeMillis();
        for (int i=0; i<repeats; i++) test = (Math.abs(testval) < 0.1)?0:testval;

        long after_mathabs =  System.nanoTime();

        long before_conditional = System.nanoTime();
        for (int i=0; i<repeats; i++) test = (testval>-0.1 && testval<0.1)?0:testval;
        long after_conditional =  System.nanoTime();


        System.out.println("Math.abs() time: " + (after_mathabs-before_mathabs));
        System.out.println("gt + lt time: " + (after_conditional-before_conditional));

    }
}
