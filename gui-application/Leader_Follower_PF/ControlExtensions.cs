using System.Reflection;
using System.Windows.Forms;

public static class ControlExtensions
{
    public static void DoubleBuffered(this Control control, bool enable)
    {
        typeof(Control).GetProperty("DoubleBuffered", BindingFlags.Instance | BindingFlags.NonPublic)
            ?.SetValue(control, enable, null);
    }
}
