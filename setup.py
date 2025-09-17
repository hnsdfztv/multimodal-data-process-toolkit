from setuptools import setup, find_packages
from pathlib import Path
import os

# 读取README文件
current_directory = Path(__file__).parent
long_description = "多模态数据工具链 - 跨平台数据收集与KITTI格式转换工具包"

# 读取requirements.txt（过滤注释和空行）
def read_requirements():
    requirements_path = current_directory / "requirements.txt"
    if requirements_path.exists():
        with open(requirements_path, 'r', encoding='utf-8') as f:
            requirements = []
            for line in f:
                line = line.strip()
                # 跳过注释行、空行和带有#的行
                if line and not line.startswith('#') and '#' not in line:
                    requirements.append(line)
            return requirements
    return []

# 读取版本信息
def get_version():
    # 从toolkit/__init__.py读取版本号
    version_file = current_directory / "toolkit" / "__init__.py"
    if version_file.exists():
        with open(version_file, 'r', encoding='utf-8') as f:
            for line in f:
                if line.startswith('__version__'):
                    return line.split('=')[1].strip().strip('"').strip("'")
    return "1.0.0"

setup(
    # 基本信息
    name="multimodal-data-toolkit",
    version=get_version(),
    author="Multimodal Data Toolkit Team", 
    author_email="your.email@example.com",  # 请替换为实际邮箱
    description="跨平台多模态数据收集与KITTI格式转换工具包",
    long_description=long_description,
    long_description_content_type="text/plain",
    
    # 项目URL
    url="https://github.com/your-username/multimodal-data-toolkit",  # 请替换为实际URL
    project_urls={
        "Bug Tracker": "https://github.com/your-username/multimodal-data-toolkit/issues",
        "Documentation": "https://github.com/your-username/multimodal-data-toolkit/wiki",
        "Source Code": "https://github.com/your-username/multimodal-data-toolkit",
    },
    
    # 分类信息
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "Intended Audience :: Science/Research",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
        "Topic :: Scientific/Engineering :: Image Processing", 
        "Topic :: Software Development :: Libraries :: Python Modules",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Operating System :: OS Independent",
        "Environment :: Console",
    ],
    
    # 包信息
    packages=find_packages(include=['toolkit', 'toolkit.*']),
    python_requires=">=3.8",
    
    # 依赖项
    install_requires=read_requirements(),
    
    # 可选依赖项
    extras_require={
        'dev': [
            'pytest>=6.0.0',
            'flake8>=3.8.0',
            'black>=21.0.0',
            'mypy>=0.900',
        ],
        'docs': [
            'sphinx>=4.0.0',
            'sphinx-rtd-theme>=0.5.0',
        ],
        'visualization': [
            'matplotlib>=3.3.0',
            'tqdm>=4.60.0',
        ],
        'advanced': [
            'open3d>=0.13.0',  # 高级点云处理
        ]
    },
    
    # 入口点
    entry_points={
        'console_scripts': [
            'multimodal-data-toolkit=run_processing:main',
            'mdt-process=run_processing:main',
        ],
    },
    
    # 包含的数据文件
    package_data={
        'toolkit': [
            'config/*.yaml',
            'config/*.json',
        ],
    },
    
    # 额外的数据文件
    data_files=[
        ('config', ['config/default_config.yaml']),
        ('docs', ['README.md'] if Path('README.md').exists() else []),
    ],
    
    # 关键词
    keywords=[
        "robotics", "computer-vision", "dataset", "kitti", 
        "ros2", "carla", "lidar", "camera", "autonomous-driving",
        "multimodal", "data-processing", "simulation"
    ],
    
    # 许可证
    license="MIT",
    
    # 包含所有文件
    include_package_data=True,
    
    # ZIP安全
    zip_safe=False,
)
