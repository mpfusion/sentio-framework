int _getpid()
{
return 0;
}

void _exit( int status )
{
(void)status;
}

int _kill( int pid, int sig )
{
(void)pid; (void)sig;
return -1;
}

int _sbrk()
{
return 0;
}
