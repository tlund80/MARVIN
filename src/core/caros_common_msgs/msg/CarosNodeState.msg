# state of node (INIT,STOPPED,RUNNING,ERROR,FATALERROR)
string state

bool inError
string errorMsg
int64 errorCode

# true if 
bool changedEvent
string previousState
