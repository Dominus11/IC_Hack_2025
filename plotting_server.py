from flask import Flask, request, render_template_string, jsonify
from bokeh.plotting import figure
from bokeh.embed import json_item
from bokeh.resources import CDN
import json

app = Flask(__name__)

plot = figure(title="Real-time Data Plot")
scatter_renderer = plot.scatter(x=[6], y=[4], name="scatter_plot")

@app.route('/', methods=['GET'])
def index():
    return render_template_string('''
        <!DOCTYPE html>
        <html>
        <head>
            <title>Real-time Plot</title>
            {{ resources|safe }}
            <script>
                var source;
                function load_plot() {
                    fetch('/plot')
                        .then(response => response.json())
                        .then(item => Bokeh.embed.embed_item(item, "myplot"))
                        .then(() => {
                            source = Bokeh.documents[0].get_model_by_name('scatter_plot').data_source;
                        });
                }
                function add_point() {
                    var x = document.getElementById('x_input').value;
                    var y = document.getElementById('y_input').value;
                    fetch('/data', {
                        method: 'POST',
                        headers: {
                            'Content-Type': 'application/json',
                        },
                        body: JSON.stringify({xs: [parseFloat(x)], ys: [parseFloat(y)]}),
                    })
                    .then(response => response.json())
                    .then(data => {
                        source.data.x = source.data.x.concat(data.xs);
                        source.data.y = source.data.y.concat(data.ys);
                        source.change.emit();
                    });
                }
            </script>
        </head>
        <body onload="load_plot()">
            <div id="myplot"></div>
            <br>
            X: <input type="number" id="x_input">
            Y: <input type="number" id="y_input">
            <button onclick="add_point()">Add Point</button>
        </body>
        </html>
    ''', resources=CDN.render())

@app.route('/plot')
def plot_json():
    return json.dumps(json_item(plot, "myplot"))

@app.route('/data', methods=['POST'])
def add_data():
    data = request.json
    x_values = list(map(float, data['xs']))
    y_values = list(map(float, data['ys']))
    for i in range(1, len(x_values)):
        x_values[i] -= x_values[0]
        y_values[i] -= y_values[0]
    x_values[0] = 0
    y_values[0] = 0
    scatter_renderer.data_source.data = dict(x=x_values, y=y_values)
    return jsonify(data), 200

if __name__ == '__main__':
    app.run(debug=True)
