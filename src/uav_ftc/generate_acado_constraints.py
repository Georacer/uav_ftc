
# variables_strings = ['phi', 'theta', 'Va', 'alpha', 'beta', 'r']
variables_strings = ['theta', 'Va']
num_vars = len(variables_strings)

num_constraints = 4

string = ''

for i in range(num_constraints):
    string += 'ocp.subjectTo( '
    line_string = ''
    terms = [None] * (num_vars + 1)
    for j in range(num_vars):
        term = 'a{}[{}]*{}'.format(i, j, variables_strings[j])
        term_format = '{{:>{}s}}'.format(len(variables_strings[j])+7)
        terms[j] = term_format.format(term)
    final_term = 'a{}[{}]'.format(i, j+1)
    terms[-1] = '{:>6s}'.format(final_term)
    line_string += ' + '.join(terms)
    line_string += ' <= 0 );\n'
    string += line_string

print(string)