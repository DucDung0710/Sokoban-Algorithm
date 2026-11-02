"""Kiem tra solution.py khong vi pham quy dinh de bai truoc khi merge.

Phan tich tinh bang ast: import chi duoc la thu vien chuan hoac framework (sokoban, state);
cam luong/tien trinh, mang, va cac loi goi __import__, exec, eval, open, os.system...
Chay: python check_imports.py [duong_dan]   (mac dinh solution.py). Ma thoat 1 neu vi pham.
"""
import ast
import sys

FRAMEWORK = {"sokoban", "state"}
BANNED_MODULES = {"threading", "multiprocessing", "concurrent", "subprocess", "socket",
                  "urllib", "http", "ssl", "asyncio", "ftplib", "smtplib"}
BANNED_CALLS = {"__import__", "exec", "eval", "open", "import_module", "system", "popen"}


def check(path="solution.py"):
    """Tra danh sach loi dang 'file:dong: mo ta'; rong neu hop le."""
    with open(path, encoding="utf-8") as f:
        tree = ast.parse(f.read(), filename=path)
    errors = []
    for node in ast.walk(tree):
        names = []
        if isinstance(node, ast.Import):
            names = [a.name.split(".")[0] for a in node.names]
        elif isinstance(node, ast.ImportFrom):
            names = [(node.module or "").split(".")[0]]
        for n in names:
            if n in BANNED_MODULES or (n not in sys.stdlib_module_names and n not in FRAMEWORK):
                errors.append(f"{path}:{node.lineno}: import bi cam: {n}")
        if isinstance(node, ast.Call):
            f = node.func
            name = f.id if isinstance(f, ast.Name) else getattr(f, "attr", "")
            if name in BANNED_CALLS:
                errors.append(f"{path}:{node.lineno}: loi goi bi cam: {name}()")
    return errors


if __name__ == "__main__":
    errs = check(*sys.argv[1:])
    print("\n".join(errs) or "check_imports: OK")
    sys.exit(1 if errs else 0)
