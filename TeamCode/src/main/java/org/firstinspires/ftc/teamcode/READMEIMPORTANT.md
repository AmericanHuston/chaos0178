We need to make a set of functions to tell us if certain componentes
are busy. 

public boolean isBusy(<arm,slider,claw> //Probably not like this, due to ranges){
switch(<arm,slider,claw>):
    ...
    if (currentPos > minRange || currentPos < maxRange){
        return true;
    }else{
        return false;
    }
    ...
}