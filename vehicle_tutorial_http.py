
def setup_http():
    return {
        'command': ['python3', '-m', 'http.server', '{port}'],
        'absolute_url': False,
        'timeout' : 20.0,
        'launcher_entry': {
            'enabled': True,
            'title': 'Vehicle Control Tutorial'
        }
    }

