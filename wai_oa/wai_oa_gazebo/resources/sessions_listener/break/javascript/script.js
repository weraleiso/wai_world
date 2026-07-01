var i_slide_number=1;

function UpdateSlideNumber()
{
    document.getElementById('txt_filename').value=GetFilenameCurrent();
}
function UpdateDate()
{
    const dat_today=new Date();
    document.getElementById('btn_date').innerText=dat_today.toDateString();
}
function GetFilenameCurrent()
{
    var s_filepath_split=location.pathname.split('/');
    var [s_filename]=s_filepath_split[s_filepath_split.length-1].split('.');
    return s_filename;
}

function GotoSlideNumber()
{
    var s_location="";
    var s_filename=window.prompt("Slide Number:",GetFilenameCurrent())
    if(parseInt(s_filename)>0)
    {
        s_location=s_filename;
        window.location.href=s_location+".html";
    }
}

function GotoNext()
{
    var s_filename=GetFilenameCurrent();
    s_location=parseInt(s_filename)+1;
    window.location.href=s_location+".html";
}

function GotoPrevious()
{
    var s_location="";
    var s_filename=GetFilenameCurrent();
    if((parseInt(s_filename))>1)
    {
        s_location=parseInt(s_filename)-1;
        window.location.href=s_location+".html";
    }
}

function GotoTitle()
{
    window.location.href="1.html";
}

function GotoOverview()
{
    window.location.href="2.html";
}

function GotoBibliography()
{
    window.location.href="3.html";
}

function GotoCalendar()
{
    window.open('https://www.timeanddate.com/calendar/','_blank');
}
